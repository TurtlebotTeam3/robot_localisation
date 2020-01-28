import rospy
import numpy as np
import math

from robot_localisation.srv import Localise, LocaliseResponse
from geometry_msgs.msg._Twist import Twist
from geometry_msgs.msg._PoseWithCovarianceStamped import PoseWithCovarianceStamped
from sensor_msgs.msg._LaserScan import LaserScan
from std_srvs.srv import Empty, EmptyRequest

class LocaliseRobot:

    def __init__(self):
        rospy.init_node('robot_localisation', anonymous=True)
        self.rate = rospy.Rate(20)

        self.epsilon = 1.0
        self.area_ellips = np.Infinity

        self.lidar_data = []

        # --- Publisher ---
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # --- Subscriber ---
        self.scan_subscriber = rospy.Subscriber('scan', LaserScan, self._handle_scan)
        self.amcl_pose_subscriber = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self._handle_amcl_pose)

        # --- Service ---
        rospy.wait_for_service('global_localization')
        self.global_localisation = rospy.ServiceProxy('global_localization', Empty)

        self.localise_robot_service = rospy.Service('localise_robot_service', Localise, self._handle_localise)

        #rospy.spin()

    def run(self):

        while len(self.lidar_data) == 0:
            self.rate.sleep()
        self._do_localisation()

    def _handle_localise(self, data):
        """
        Handels the request for localisation.
        Starts the localisation behaviour and retruns True when the localisation is finished
        """
        result = self._do_localisation()
        return LocaliseResponse(result)
        
    def _do_localisation(self): 
        # Initiate global localization, wherein all particles are dispersed randomly through the free space in the map.
        self.global_localisation()

        while self.area_ellips > self.epsilon:
            range_front = []
            range_front[:20] = self.lidar_data[-20:]
            range_front[20:] = self.lidar_data[:20]

            obstacle_in_front = self._is_obstacle_in_front()
            if obstacle_in_front:
                # rotate to the right
                self._move(0, -0.5)
            else:
                # move straight forward
                self._move(0.25, 0)

        self._move(0, 0)
        return True

    def _is_obstacle_in_front(self):
        """
        Detect if there is a obstacle in front
        """
        range_front = []
        range_front[:20] = self.lidar_data[-20:]
        range_front[20:] = self.lidar_data[:20]
        range_front = list(filter(lambda num: num != 0, range_front))
        min_front = min(range_front)
        if min_front < 0.4 and min_front != 0.0:
			return True
        else:
			return False

    def _move(self, linear, angluar):
        """
        Set the motor speeds
        """
        vel_msg = Twist()
		# Linear velocity in the x-axis.
        vel_msg.linear.x = linear
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

		# Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angluar
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()
    
    def _handle_scan(self, data):
        """
        Handles the data published to the scan topic of the lidar
        """
        self.lidar_data = data.ranges
    
    def _handle_amcl_pose(self, data):
        """
        Handles data published to the amcl_pose topic
        """
        cov = data.pose.covariance
        cov = np.reshape(cov,(6,6))

        a, b, deg = self._calc_ellipse(cov)

        self.area_ellips = a * b * math.pi

    def _calc_ellipse(self, cov):
        """
        calculation error ellips
        """
        cov1 = cov[0:2, 0:2]
        eig_val, eig_vec = np.linalg.eig(np.linalg.inv(cov1))
        eigen_x, eigen_y = eig_vec[:, 0]
        deg = np.degrees(np.arctan2(eigen_y.real, eigen_x.real))
        a, b = 2 / np.sqrt(eig_val)
        return a, b, deg

if __name__ == "__main__":
    try:
        lr = LocaliseRobot()
        lr.run()
    except rospy.ROSInterruptException:
        pass