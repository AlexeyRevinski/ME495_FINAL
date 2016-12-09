# ME495_FINAL
Fancy Dinner


## PROBLEM STATEMENT
The objective of this project is a simple setup for a fancy dinner using extensive capabilities of Baxter, our loyal servant (in both meanings of the word). Given a set of three utensils (a fork, a spoon, and a knife), Baxter will be required to place them in a specified position relative to a plate (fixed in the center), akin to proper table setting in a fancy restaurant. Each utensil is marked with a different color (red, green, or blue), which allows Baxter to differentiate between the utensils and pick and place them in their proper positions. In this demonstration, Baxter also determines the position of the plate and stores it for future reference. 

## WHAT YOU NEED TO RUN THIS

## PHYSICAL SETUP
For this demo, you will need the following:

1. *Black table cloth*: to prevent the reflection of the table from affecting Baxter's white balance and exposure, which consequently may affect its image processing capabilities of certain (or all) colors.
2. *Black plate*: black is preferred for this set-up due to the reasons described above.
3. *Utensils*: fork, knife, spoon.
4. *Color tape*: for this demo, we used __green, blue,__ and __red__ tape. It was determined that these three colors were easiest for Baxter to process and differentiate with its hand cameras. We taped each color to the largest area of each utensil and taped the center of the plate with red. For image processing, it is important for each color to cover enough area for Baxter to recognize with its cameras.
5. *Foam padding*: to tape to Baxter's right hand flat grippers to ensure thin utensils are gripped securely.
6. *Utensil stand*: we custom-made a utensil stand for Baxter to easily pick up thin utensils from the table.


## HIGH LEVEL DESCRIPTION OF THE NODES/SERVICES
__`dapper_baxter.py`__


__`ik_node.py`__


__`me495_baxter_reset.py`__

This node, after initializing a node and enabling Baxter, instantiates two Limb objects, for the left and right arms and a Head object. Using its move_to_neutral() class function we set the left and right limbs back to the neutral pose. Using the head.set_pan() class function, we reset the pan angle back to 0. 

__`opencv_right.py`__

This node handles the color and circle center detection (center is key to tracking the target utensil's position). The high level description of this node operation is as follows: ROS Image message types are converted through CvBridge to openCV images. By using preset HSV minimum/maximum boundaries, as well as preset exposure settings, Baxter's right hand camera can isolate the positions of the red regions and apply a binary mask with the cv2.threshold functionality. Using the region that are boolean "True" (e.g. red in HSV coordinates), a miniumum enclosing circle (x,y,radius) is drawn on the "True" regions using cv2.findContours and cv2.minEnclosingCircle. With respect to ROS communication, this node acts as a subscriber to the "/robot/limb/right/endpoint_state" and "/cameras/right_hand_camera/image" topics, while also initializing a service that takes in the detected color and returns the (x,y) coordinates of the region of interest. 

## FINAL FUNCTIONALITY (VIDEO)

## ISSUES AND WORKAROUNDS
__Image Processing__

1. Exposure
Baxter's image processing depends highly on its ability to recognize different colors. Due to the effects the external environment (*i.e.,* lighting) may have on the way its cameras perseive color, it is important to callibrate them before running this demo. After running this demonstration under two different lighting conditions (on two separate Baxters), it was determined that controlling the exposure of Baxter's hand cameras may have a great impact on whether it is able to process certain colors or not. It was also determined that the same color (in our case, it was red) will be perceived differently by its cameras depending on the height of the camera placed above each color. In this case, it is possible that Baxter will pick up red at one height, but will not see it at another height. To solve this, we changed the exposure of the hand camera for each condition using the following line in `dapper_baxter.py`:
```
camera = baxter_interface.camera.CameraController('right_hand_camera')
camera.exposure =9
```

2. HSV range
Baxter's image processing also depends on the HSV range you choose for each color and is also greatly affected by the external environment Baxter is in. To solve issues arising from incorrect HSV range, we tested color recognition for each color by manually changing the HSV range for each color. 

3. Callibration value

