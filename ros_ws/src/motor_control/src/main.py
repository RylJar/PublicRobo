#!/usr/bin/env python3
from motors import MotorControl
import rospy


class Main:
    def __call__(self):
        rospy.init_node('motor_control')
        motot_control = MotorControl()
        motot_control.control_loop()

if __name__ == "__main__":
    main = Main()
    main()
    rospy.spin()
