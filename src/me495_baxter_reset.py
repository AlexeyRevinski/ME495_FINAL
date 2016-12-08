#!/usr/bin/env python
import sys
import rospy
import baxter_interface


def main():
    rospy.init_node("me495_baxter_reset")
    rate = rospy.Rate(10)
    baxter_interface.RobotEnable().enable()
    head = baxter_interface.Head()
    rightlimb = baxter_interface.Limb('right')
    leftlimb = baxter_interface.Limb('left')
    leftlimb.move_to_neutral()
    rightlimb.move_to_neutral()
    head.set_pan(0.0)
    
if __name__ == "__main__":
    sys.exit(main())
