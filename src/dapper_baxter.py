#!/usr/bin/env python
import sys
import rospy
import baxter_interface
from example.srv import ArmMovement
from example.srv import image_proc

def calibrate():
    zval = -1
    rospy.set_param('/ZOFF',-1)
    while zval == -1 and not rospy.is_shutdown():
        zval = rospy.get_param('/ZOFF')
    return zval
    
def baxter_dab():
    whichlimb = baxter_interface.Limb('right')
    head = baxter_interface.Head()
    move_arm = rospy.ServiceProxy('/handle_ik',ArmMovement)
    angles = whichlimb.joint_angles()
    angles['right_e1']=0.0
    angles['right_w1']=0.0
    angles['right_s1']=-0.4
    angles['right_s0']=-0.8
    whichlimb.move_to_joint_positions(angles)
    x = 0.4
    y = 0.0
    z = 0.15
    limb = 'left'
    move_arm(x,y,z,limb)
    head.set_pan(0.7)
    rospy.sleep(3)
    
def main():
################################################################################
    # INITIALIZE NODE
    rospy.init_node("da2pper_baxter")
    rate = rospy.Rate(10)
    # ENABLE BAXTER
    baxter_interface.RobotEnable().enable()
    gripper = baxter_interface.Gripper('right')
    head = baxter_interface.Head()
    head.set_pan(0.0)
    # SET UP IK SERVICE
    rospy.wait_for_service('/handle_ik')
    move_arm = rospy.ServiceProxy('/handle_ik',ArmMovement)
    # SET UP IMAGE PROCESSING SERVICE
    rospy.wait_for_service('/handle_image_proc_right')
    my_color_serv_right = rospy.ServiceProxy('/handle_image_proc_right',image_proc)
    # TUNE THE CAMERA
    camera = baxter_interface.camera.CameraController('right_hand_camera')
    camera.exposure =9#12
    camera.resolution = (640,400)
    # REFRESH PARAMETERS
    rospy.set_param('/xb',0)
    rospy.set_param('/yb',0)
    rospy.set_param('/Bpx',0)
    rospy.set_param('/Bpy',0)
    rospy.delete_param('/xb')
    rospy.delete_param('/yb')
    rospy.delete_param('/Bpx')
    rospy.delete_param('/Bpy')
    rospy.set_param("/try_again",0)
    # OPEN GRIPPER
    gripper.calibrate()
    gripper.open()
    # WAIT FOR EVERYTHING TO INITIALIZE
    rospy.sleep(1)
################################################################################
 
    # STANDARD VALUES
    z_stand_to_plate = 0.1
    x_home = 0.70
    y_home = 0.00
    z_home = 0.15
    x_plate = 0.6
    y_plate = -0.4
    z_plate = z_home-z_stand_to_plate
    # GO HOME
    x = x_home
    y = y_home
    z = z_home
    limb = 'right'
    move_arm(x,y,z,limb)
    
    # CALIBRATE HEIGHT
    z_off = calibrate()
    rospy.loginfo("CALIBRATED")
    rospy.set_param("zoff",z_off)
    
    # DETECT PLATE PLACEMENT
    x = x_plate
    y = y_plate
    z = z_plate
    limb = 'right'
    move_arm(x,y,z,limb)
    camera.exposure = 5
    rospy.sleep(2)
    color = 'red' 
    my_color_serv_right(color)
    rospy.sleep(2)
    plate_pos_x = rospy.get_param('/xb')
    plate_pos_y = rospy.get_param('/yb')
    
    # RETURN HOME
    x = x_home
    y = y_home
    z = z_home
    limb = 'right'
    move_arm(x,y,z,limb)
    camera.exposure = 9
    rospy.sleep(2)
###############################################################################      
    
    
    
    # SRART PICKING AND PLACING
    color = 'blue'
    my_color_serv_right(color)
    rospy.sleep(2)
    x = rospy.get_param('/xb')-0.08
    y = rospy.get_param('/yb')
    z = z_home-z_off
    move_arm(x,y,z,limb)
    if rospy.get_param("/try_again")==1:
        move_arm(x,y,z,limb)
        rospy.set_param("/try_again",0)
    gripper.close()
    rospy.sleep(0.5)
    x = x_home
    y = y_home
    z = z_home
    move_arm(x,y,z,limb)
    x = plate_pos_x
    y = plate_pos_y + 0.15
    z = z_home-z_off-0.05
    move_arm(x,y,z,limb)
    gripper.open()
    rospy.sleep(0.25)
    rospy.sleep(0.5)
    x = x_home
    y = y_home
    z = z_home
    move_arm(x,y,z,limb)
    rospy.sleep(2)
    
    color = 'green'
    my_color_serv_right(color)
    rospy.sleep(2)
    x = rospy.get_param('/xb')-0.08
    y = rospy.get_param('/yb')
    z = z_home-z_off
    move_arm(x,y,z,limb)
    if rospy.get_param("/try_again")==1:
        move_arm(x,y,z,limb)
        rospy.set_param("/try_again",0)
    gripper.close()
    rospy.sleep(0.5)
    x = x_home
    y = y_home
    z = z_home
    move_arm(x,y,z,limb)
    x = plate_pos_x
    y = plate_pos_y-0.15
    z = z_home-z_off-0.05
    move_arm(x,y,z,limb)
    gripper.open()
    rospy.sleep(0.25)
    rospy.sleep(0.5)
    x = x_home
    y = y_home
    z = z_home
    move_arm(x,y,z,limb)
    rospy.sleep(2)
    
    color = 'red'
    my_color_serv_right(color)
    rospy.sleep(2)
    x = rospy.get_param('/xb')-0.08
    y = rospy.get_param('/yb')
    z = z_home-z_off
    move_arm(x,y,z,limb)
    if rospy.get_param("/try_again")==1:
        move_arm(x,y,z,limb)
        rospy.set_param("/try_again",0)
    gripper.close()
    rospy.sleep(0.5)
    x = x_home
    y = y_home
    z = z_home
    move_arm(x,y,z,limb)
    x = plate_pos_x
    y = plate_pos_y-0.20
    z = z_home-z_off-0.05
    move_arm(x,y,z,limb)
    gripper.open()
    rospy.sleep(0.25)
    rospy.sleep(0.5)
    x = x_home
    y = y_home
    z = z_home
    move_arm(x,y,z,limb)
    rospy.sleep(2)
    
    baxter_dab()   


if __name__ == "__main__":
    sys.exit(main())
