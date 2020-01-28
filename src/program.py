import rospy

from robot_localisation.srv import Localise

class LocaliseRobot:

    def __init__(self):
        rospy.init_node('robot_localisation', anonymous=True)

        self.localise_robot_service = rospy.Service('localise_robot_service', Localise, self._handle_localise)


    def _handle_localise(self, data):
        pass
    
if if __name__ == "__main__":
    try:
        lr = LocaliseRobot()
    except rospy.ROSInterruptException:
        pass