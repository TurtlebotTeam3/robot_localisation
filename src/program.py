#! /usr/bin/env python
import rospy
import numpy as np
import math

from robot_localisation.srv import Localise, LocaliseResponse
from geometry_msgs.msg._Twist import Twist
from geometry_msgs.msg._PoseWithCovarianceStamped import PoseWithCovarianceStamped
from sensor_msgs.msg._LaserScan import LaserScan
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import Bool

class LocaliseRobot:

    def __init__(self):
        rospy.init_node('robot_localisation', anonymous=True)
        self.rate = rospy.Rate(20)

        self.epsilon = 0.4
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

        rospy.loginfo("---ready---")
        rospy.spin()

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
        return_vel = Bool()
        return_vel.data = result
        return LocaliseResponse(return_vel)
        
    def _do_localisation(self): 
        # Initiate global localization, wherein all particles are dispersed randomly through the free space in the map.
        self.global_localisation()

        #wait for pointcloud being destributet over the map
        count = 0
        while count < 50:
            self.rate.sleep()
            count = count + 1

        move_straight_count = 0
        while self.area_ellips > self.epsilon:
            range_front = []
            range_front[:20] = self.lidar_data[-20:]
            range_front[20:] = self.lidar_data[:20]

            obstacle_in_front = self._is_obstacle_in_front()
            if obstacle_in_front:
                # rotate to the right
                self._move(0, -0.75)
            else:
                if move_straight_count % 100 == 0:
                    self._rotate_x_degrees(60, 360, True) 

                # move straight forward
                move_straight_count = move_straight_count + 1
                self._move(0.25, 0)

        self._move(0, 0)
        return True
    
    def _rotate_x_degrees(self, speed_degrees_sec, rotate_in_degrees, clockwise):
        """
        Rotate robot
        """
        vel_msg = Twist()

        #Converting from angles to radians
        angular_speed = speed_degrees_sec*2*math.pi/360
        relative_angle = rotate_in_degrees*2*math.pi/360

        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        # Checking if our movement is CW or CCW
        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)
        
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < relative_angle):
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        #Forcing our robot to stop
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()    

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
        if cov != None and len(data.pose.covariance) == 36:
            try:
                cov = np.reshape(cov,(6,6))
                if cov.shape[0] == 6 and cov.shape[1] == 6:
                    a, b, _ = self._calc_ellipse(cov)

                    self.area_ellips = a * b * math.pi
                else:
                    rospy.loginfo("shape wrong")
            except:
                rospy.loginfo("covariance exception")
        else:
            rospy.logerr("wrong length of array")

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
    except rospy.ROSInterruptException:
        pass