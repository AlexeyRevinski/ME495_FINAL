#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from baxter_core_msgs.msg import EndpointState
from example.srv import image_proc


bridge = CvBridge() #Creates obj for sending poop btwn ros and opencv

def imagecb(data):
    global bridge

    #Converts Image message to CV image with blue-green-red color order (bgr8)
    try: #if no image, then throw out error but continue
        img_original = bridge.imgmsg_to_cv2(data, "bgr8")


#COLOR TRACKING
        #Converts bgr8 to hsv
        hsv = cv2.cvtColor(img_original, cv2.COLOR_BGR2HSV)

        #Def range of red color in HSV
        color_dict = {
            'blue':{
                'lower':np.array([100,50,0]),
                'upper':np.array([115,255,255])
            },
            'red':{
                #'lower':np.array([-50,100,100]),
                #'upper':np.array([10,255,255])
                'lower':np.array([-50,100,0]),
                'upper':np.array([10,200,255])
            },
            'purple': {
                'lower':np.array([260,50,25]),
                'upper':np.array([360,255,125])
            },
            'green': {
                'lower':np.array([60,85,0]),
                'upper':np.array([90,200,255])
            },
            'brown': {
                'lower':np.array([0,100,100]),
                'upper':np.array([0,255,255])
            }
        }
        #Def mask using set hsv range
        try:
            mask = cv2.inRange(hsv,color_dict[color]['lower'] ,color_dict[color]['upper'])
        except:
            #mask = cv2.inRange(hsv,color_dict['blue']['lower'] ,color_dict['blue']['upper'])
            #mask = cv2.inRange(hsv,color_dict['green']['lower'] ,color_dict['green']['upper'])
            mask = cv2.inRange(hsv,color_dict['red']['lower'] ,color_dict['red']['upper'])
            #mask = cv2.inRange(hsv,color_dict['purple']['lower'] ,color_dict['purple']['upper'])
            #mask = cv2.inRange(hsv,color_dict['brown']['lower'] ,color_dict['brown']['upper'])
        mask = cv2.erode(mask,None,iterations=5)
        mask = cv2.dilate(mask,None,iterations=5)

        #Mask and original image overlay
        res = cv2.bitwise_and(img_original,img_original, mask= mask)

        #Creating b w image from res  (outputs binary matrices)
        ret, thresh = cv2.threshold(res[:,:,2], 100, 255, cv2.THRESH_BINARY)


        #Find and creat the circle
        cnts = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        if len(cnts) > 0:
            c=max(cnts,key=cv2.contourArea)
            ((x,y),radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            print center
            height,width,depth=img_original.shape
            if radius > 10:

                cv2.circle(img_original,(int(x), int(y)), int(radius), (0,255,255),2)
                cv2.circle(img_original,center,5,(0,0,255),-1)

            #Have to convert pixel points to 3D points for baxter as follows
            calibration = 0.0032#0.0023
            cam_disp_x = 0.04
            cam_disp_y = -0.02
            z_disp = rospy.get_param('/zoff')
            cx = center[0]
            cy = center[1]
            xb = (cy - 0.5*height)*calibration*z_disp + Bpx + cam_disp_x
            yb = (cx - 0.5*width)*calibration*z_disp + Bpy + cam_disp_y
            rospy.set_param('/xb',xb)
            rospy.set_param('/yb',yb)
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.loginfo((xb,yb))
            rospy.set_param('/Bpx',Bpx)
            rospy.set_param('/Bpy',Bpy)
            
#DISPLAY WHAT CAMERA IS PUBLISHING TO opencv2
        #cv2.imshow("colorout", res)
        #cv2.imshow("contout", thresh)
        cv2.imshow("original",img_original)

        cv2.waitKey(20) #Updates with next cached img every 0.01sec

    except CvBridgeError, e:
        print("==[CAMERA MANAGER]==", e)


def get_body_stuff(data):
    global Bpx, Bpy
    Bpx = data.pose.position.x
    Bpy = data.pose.position.y
    Bpz = data.pose.position.z
    rospy.set_param('/Bpz',Bpz)


def define_color(data):
    global color
    color = data.color
    return 0

def listener():
    rospy.init_node('listener',anonymous=True)
    #Initializes node
    #create a publisher to publish xb and yb
    #subscribe to color topic
    
    rospy.Service('handle_image_proc_right',image_proc,define_color)
    rospy.Subscriber("/robot/limb/right/endpoint_state",EndpointState,get_body_stuff)
    rospy.Subscriber("/cameras/right_hand_camera/image",Image,imagecb)
    
    #Def node as subscriber with rostopic
    rospy.spin()
    #Loops python until node stopped
    #Let's create a service that takes in color and returns the x and y

if __name__ == '__main__':
    global xb,yb
    rospy.set_param('/zoff',0)
    listener()
