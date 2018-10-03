[//]: # (Image References)

[imageRobotConfiguration]: ./WriteupImages/imageRobotConfiguration.jpg
[imageDHTransform]: ./WriteupImages/imageDHTransform.png
[imageDHParameters]: ./WriteupImages/imageDHParameters.pnh
[imageHomogenousTransform]: ./misc_images/imageHomogenousTransform.png
[imageTransformApplication]: ./misc_images/imageTransformApplication.png
[imageFKEquation]: ./misc_images/imageFKEquation.png
[imageWristCenterEquation]: ./misc_images/imageWristCenterEquation.png
[imageFirstAngleDerivation]: ./misc_images/imageFirstAngleDerivation.jpg
[imageFirstAngleDerivation2]: ./misc_images/imageFirstAngleDerivation2.jpg
[imageFirstAngleFormation]: ./misc_images/imageFirstAngleFormation.jpg
[imageSecondAngleDerivation]: ./misc_images/imageSecondAngleDerivation.jpg
[imagePickAndPlace]: ./misc_images/imagePickAndPlace.png

## Project: Kinematics Pick & Place

**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 

---
### Project Summary
The pick and place project is modeled after the Amazon Robotics Challenge. In its simplest form, the challenge is to program the KUKA KR210 to pick heup an object from a known location on a shelf and to drop it in a bin located at another known location. The KUKA KR210 robot is made up of 6 revolute joints. The end-effector that does the grips the object is rigidly attached to frame 6. The target position and orientation of the end-effector is supplied to our algorithm as inputs. the aim is to calculate the angle of rotation of each revolute joint in order to achieve the required end-effector position and orientation. There are two parts to solving this problem. First we understand the forward kinematics of the robot - given the joint angles, what is the position and orientation of the end effector. Once we understand this part, we will use our findings in the inverse kinematics problem to find joint angles, given the end effector pose.

### Kinematic Analysis
#### 1. Forward Kinematic Analysis
The forward kinematics problem answers the question "What is the end effector pose, given the robot's joint angles". The steps involved in solving the forward kinematics problem:
1. From the robot's joint configuration at zero joint angles, understand how each joint is linked to the next one. In order to understand joint relations, we first assign coordinate frames to each joint and then figure out the orientation and offset between the frame's axes. Particularly, there are four parameters per joint:
alpha_(i-1): Angle between the z axis of link (i-1) and link (i)
a_(i-1): Distance between the z axis of link (i-1) and link (i)
d_(i): Distance between the x axis of link (i-1) and link (i)
theta_(i): Angle between the x axis of link (i-1) and link (i)
The robot's coordinate frames and some of the angles are illustrated in the figure below: 
![alt text][imageRobotConfiguration]

These parameters are known as the "Denavit-Hartenberg" parameters and are summarized in a table below.
![alt text][imageDHParameters]


2. Once we have the joint parameters, we figure out the homogenous transformation matrix between each joint. The homogeneous transformation matrix is a result of the equation
![alt text][imageDHTransform]

Where:
R(x_(i-1), alpha_(i-1)) is the rotation matrix between the x axis of links (i-1) and (i)
T(x_(i-1), alpha_(i-1)) is the translation matrix between the x axis of links (i-1) and (i)
R(z_(i), theta_(i)) is the rotation matrix between the z axis of links (i-1) and (i)
T(z_(i), theta_(i)) is the translation matrix between the z axis of links (i-1) and (i)

The general form of this homogeneous transform is:
![alt text][imageHomogenousTransform]

In the end, the transforms are multiplied together to express the end-effector pose in the base link's coordinate system. Applying the transform to each joint, we get:
![alt text][imageTransformApplication]

The end effector position can be evaluated as:
![alt text][imageFKEquation]

#### 2. Inverse Kinematic Analysis

The last three joints of the robot are revolute joints. This configuration is commonly known as a "spherical wrist". Research has shown that when a spehrical wrist is present in the robot, the inverse kinematics problem can be split into two decoupled problems - finding the first three joint angles from the end-effector position, and finding the last three joint angles from the end-effector orientation. 

In order to compute the joint angles, we first compute the wrist center. According to our robot configuration, the wrist center is placed at the intersection of the three revolute joints frame, which is basically wrist 5's origin. The wrist center can be derived by offseting the end effector position by the projection of (d6+l) on the z-frame of the DH transformation matrix:
![alt text][imageWristCenterEquation]

Where nx, ny, nz are the rotation matrix components along the z-frame of the transformation matrix.

Once we have the wrist center, a triangle is made from joint 2, joint 3 and the wrist center. The following derivations can be used to calculate the first three joint angles:
![alt text][imageFirstAngleFormation]
![alt text][imageFirstAngleDerivation]
![alt text][imageFirstAngleDerivation2]

The next step is to calculate the joint angles 4,5,6 from the end effector orientation. The overall rotation between the base link and the gripper link must be equal to the product of individual rotations. Using this, we calculate the rotation matrix between link 3 to link 6 as follows:
R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6
R0_6 = Rrpy
R3_6 = inv(R0_3) * Rrpy

Once we have the rotation matrix between links 3 and 6, we can calculate the respective angles as follows:
![alt text][imageSecondAngleDerivation]


### Project Implementation
The code to implement the equations from the analysis step are present in IK_Debug.py and in IK_server.py. The IK_Debug.py containts test cases that can be used to evalate the implementation. The first block of code implements the Forward Kinematics. The next block of code is used to find the joint angles from the wrist center. The final block of code computes the remaining joint angles from the end effector orientation. Snippets of code are given in the above explanation, and the entire code can be found in the IK_server.py file in this repository.

Here's an example of the pick and place in progress:
![alt text][imagePickAndPlace]

There are few steps that we could do to improve the code:
1. To improve the performance, we can pre-compute the homogeneous transform matrix outside the time loop, and just evaluate it at every time step.
2. To evaluate the performance of pick and place, we can calculate the error between the requested end-effector pose and the achieved end-effector pose over time (known as error curves).


