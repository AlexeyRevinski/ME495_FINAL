# ME495_FINAL
Fancy Dinner


## PROBLEM STATEMENT
The objective of this project is a simple setup for a fancy dinner using extensive capabilities of Baxter, our loyal servant (in both meanings of the word). Given a set of three utensils (a fork, a spoon, and a knife), Baxter will be required to place them in a specified position relative to a plate (fixed in the center), akin to proper table setting in a fancy restaurant. Each utensil is marked with a different color (red, green, or blue), which allows Baxter to differentiate between the utensils and pick and place them in their proper positions. In this demonstration, Baxter also determines the position of the plate and stores it for future reference. 

## WHAT YOU NEED TO RUN THIS

## PHYSICAL SETUP
For this demo, you will need the following:

1. *Black table cloth*: to prevent the light reflection from the table from affecting Baxter's white balance and exposure, which consequently may affect its image processing capabilities of certain (or all) colors.
2. *Black plate*: black is preferred for this set-up due to the reasons described above.
3. *Utensils*: fork, knife, spoon, and any other dining utensils.
4. *Color tape*: for this demo, we used __green, blue,__ and __red__ tape. It was determined that these three colors were easiest for Baxter to process and differentiate with its hand cameras. We taped each color to the largest area of each utensil and taped the center of the plate with red. For image processing, it is important for each color to cover enough area for Baxter to recognize with its cameras.
5. *Foam padding*: to tape to Baxter's right hand flat grippers to ensure thin utensils are gripped securely.
6. *Utensil stand*: we custom-built a utensil stand for Baxter to easily pick up thin utensils from the table. Since the utensils are so thin, Baxter needed some gripper clearance to get a more secure grip on the utensils. The stand was spray painted black following the reasons described above. 


## HIGH LEVEL DESCRIPTION OF THE INCLUDED FILES

### LAUNCH FILES

__`dinner.launch`__

The `dinner.launch` file launches 4 nodes: `xdisplay_image.py`,`dapper_baxter.py`,`ik_node.py`, and `opencv_right.py`. `xdisplay_image.py` loads our "fancy" image to transfrom Baxter from an ordinary robot to one with the visage of an elegant English butler, complete with monocle and well-groomed mustache. This node is provided in the `baxter_examples` package, and takes in an argument to the specified file path to the desired image. This resets the Baxter head display image (Baxter's head display is 1024x600 pixel resolution). Please see below for the description of the latter three nodes.


__`reset.launch`__

The `reset.launch` file runs the `xdisplay_image.py` node again, with a modified `args` to specify the file path to the default SDK Baxter head display image. The `me495_baxter_reset.py` node is launched afterward to reset the head pan angle, left, and right arms back to the zero position and neutral positions, respectively.

### SERVICES

__`ArmMovement.srv`__

`float32 x`,`float32 y`,`float32 z`,`string limb` are the responses in the `ArmMovement.srv`, while `bool b` is the response.

This is used in `ik_node.py` to verify if a valid set of joint angles were successfully found via Baxter's inverse kinematics service.

__`image_proc.srv`__

`string color` is the response in the `image_proc.srv`, while `bool b` is the response. 

This is used in `opencv_right.py` to verify if a color was successfully found by our openCV color detection node.

### NODES
__`dapper_baxter.py`__

This node uses the `/handle_ik` service (of type `ArmMovement.srv`) to send commands to the `ik_node.py` in order to move the robot into certain configurations. It also uses the `/handle_image_proc_right` service (of type `image_proc.srv`) to get 3D coordinates of various objects from the `opencv_right.py` node.

After initializing baxter (calibrating and opening its gripper, making camera and limb objects, etc.), the code calls on the `/ik_handle` service to move the robot into a pre-defined "home" position. Then, the robot waits for a calibration factor parameter. This parameter is the vertical (z) distance in meters from the top of the utensil stand to the vertical midpoint of the foam grippers.

When that parameter is passed by the user, the robot starts its main sequence.

That sequence starts with the robot moving to a pre-set "plate home" position, where the center of the plate is most likely to be. Information from `opencv_right.py` node is used to detect the center of the plate. Then, the robot moves to the "home configuration" and starts looking for utensils.

The code has three colors lined up for picking: blue, then green, and red. For each one, it first communicates with `opencv_right.py` to get the 3D location of an object of the particular color, then moves Baxter's right gripper to the location of that object offset a bit towards Baxter to grip the utensil somewhere closer to its center of mass (there is colored tape placed at the centers of mass of the utensils, with more tape at the ends for more robust color detection-please click the link in the Final Functionality section to our video for better visualization). The gripper is then closed, and the limb is moved into its home configuration. Then, the code moves the limb to the center-of-the-plate configuration with a hard-coded offset to the left or right of the plate to drop off the utensil. The gripper is opened, the utensil happily drops to the table, and Baxter returns to "home". Then, the process repeats for the other colors. 

In various sections of the code, we implemented rospy.sleep() commands to counter various physical and software timing issues.

At the end, baxter implements a celebratory stance that is self-explanatory from its name. For funs.

__`ik_node.py`__

This node listens to the `/handle_ik` service and utilizes Baxter's IK service to, based on received goal coordinates, solves the inverse kinematics problem and moves the commanded limb into the goal configuration. If the IK solver does not find a solution, the limb is moved to a neutral position. Please note that the orientation of the end-effector is specified by the `PoseStamped` message with the sub-field `Quarternion()`. For both the left and right arm, these are both hardcoded as ` x = 1.00, y = 0.00 z = 0.00 w = 0.00`.

__`me495_baxter_reset.py`__

This node enables Baxter, instantiates the left and right `baxter_interface.Limb()` objects and a `baxter_interface.Head()` object. Using its `.move_to_neutral()` class function we set the left and right limbs back to the neutral pose. Using the `head.set_pan()` class function, we reset the pan angle back to 0. 

__`opencv_right.py`__

This node handles the color and circle center detection (center is key to tracking the target utensil's position). ROS Image message types are converted through CvBridge to openCV images. By using preset HSV minimum/maximum boundaries, as well as preset exposure settings, Baxter's right hand camera can isolate the positions of the red regions and apply a binary mask with the `cv2.threshold` functionality. Using the region that are boolean "True" (e.g. red in HSV coordinates), a miniumum enclosing circle (x,y,radius) is drawn on the "True" regions using `cv2.findContours` and `cv2.minEnclosingCircle`. This node subscribes to the `/robot/limb/right/endpoint_state` and `/cameras/right_hand_camera/image` topics and initiates a service that takes in a color command and returns the (x,y) coordinates of the center of the region containing that color. 

## FINAL FUNCTIONALITY (VIDEO)

This [link](https://vimeo.com/195054458) goes to our video uploaded on Vimeo, depicting a successful trial run with a close up view

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

3. Calibration values

To account for the various vertical distances involved in our project implementation, we defined a reference position for Baxter's right hand end-effector to be above the utensil placeholder stage. This, coupled with variance in distances of the plate to the placeholder stage and camera calibration issues, necessitated the need to re-calculate a `ZOFF` parameter in different environments, since the distance metrics were measured by hand and hardcoded. 

Another calibration concern arose from Baxter's right hand camera. By the following formula for converting pixel coordinates to 3D coordinates with respect to the fixed world frame:

```
xb = (cy - 0.5*height)*calibration*z_disp + Bpx + cam_disp_x
yb = (cx - 0.5*width)*calibration*z_disp + Bpy + cam_disp_y
```

where (cx,cy) represent the center coordinates in pixel coordinates (collected from color and contour detection), (height,width) represent the pixel resolution of the image at the time of measurement, Bpx and Bpy being the position and orientation of Baxter's hand camera with respect to Baxter's fixed frame, and the remaining z_disp, calibration, cam_disp_x, and cam_disp_y being calibration factors. There were trials we noticed that Baxter would attempt to grip in such an offset from the desired position such that the offset was "symmetric" relative to the center of the region Baxter's right hand camera was viewing. We noticed that by tuning the "calibration" parameter, we were able to mitigate this effect assuming other variables were held constant. This proved to be more difficult than anticipated due to influence from other image processing variables such as lighting/exposure and ZOFF.

__Timing__

1. Timing Issues

Debugging with rospy.loginfo() had varying degrees of success possible timing instability, printing to the terminal adds small delay

Potential timing issues from our various nodes (queue size, publishing rate, cv2.waitforkey()) could have limited Baxter's ability to obtain the correct target positions needed to perform correct object detection and retrieval. False positives would manifest, but we noticed by adding various `rospy.loginfo()` commands, the error rate decreased; this shows there may be correlation between the timing and successful detection/retrieval. 

__Distance__

1. Errors in IK Solutions

When distances between the desired utensil positions and the utensil placeholder stage were larger than the optimal distance, there were a non-trivial number of instances IK would not solve. This initially manifested as the end-effector freezing in place after calling the IK service `move_arm(x,y,z,limb)`. We attempted to fix this by, within `ik_node.py`, creating a if_else condition to try calling the IK service. If there was no valid solution found, then the right limb would move back to the neutral position with the `move_to_neutral()` function, then send another request to the IK service to try finding a valid joint position again. This unfortunately was not a robust solution, as it is entirely possible a valid solution would not be found in the neutral configuration either.   

