## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[homo_trans]: ./misc_images/homogeneous_transform.png
[homo_trans_deriv]: ./misc_images/homogeneous_transform_deriv.png
[homo_trans_T_0_G]: ./misc_images/homogeneous_transform_T_0_G.png
[arm_default_transform]: ./misc_images/arm_default_transform.png
[ee_default_pos]: ./misc_images/ee_default_pos.png
[decoupling_matrix]: ./misc_images/decoupling_matrix.png
[wrist_center_formula]: ./misc_images/wrist_center_formula.png
[R3_6_matrix_derivation]: ./misc_images/R3_6_matrix_derivation.png
[R3_6_matrix]: ./misc_images/R3_6_matrix.png
[R0_3_matrix]: ./misc_images/R0_3_matrix.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

General assumptions that applies to the analysis:
- All rotations follow the right-hand rule.
- Lengths are given in meters and theta angles are in radians.

We first need to extract the URDF joint and link properties of intrest from the KR210 URDF description file `kr210.urdf.xacro` as follows:

joint|parent link|child link|rotation axis|dx|dy|dz|theta min|theta max
---|---|---|---|---|---|---|---|---
1|base_link|link_1|z|0|0|0.33|-185|185
2|link_1|link_2|y|0.35|0|0.42|-45|85
3|link_2|link_3|y|0|0|1.25|-210|65
4|link_3|link_4|x|0.96|0|-0.054|-350|350
5|link_4|link_5|y|0.54|0|0|-125|125
6|link_5|link_6|x|0.193|0|0|-350|350
gripper|link_6|gripper_link|-|0.11|0|0|-|-

Where:
- `dx,dy,dz` represents the translation from the origin of the parent link to the origin of the child link in which the joint sits at the origin of the child link.

Then we draw schematics for the KR210 6DOF manipulator based on the modified DH convention and the DH parameter assignment algorithm mentioned in the lectures:

*TODO*

Then from the schematics we derive the DH parameter table based on the modified DH convention and the KR210 URDF joints table above.

i|T(i-1, i)|alpha(i-1)|a(i-1)|d(i)|theta(i)
--|--|--|--|--|--
1|T(0,1)|0|0|0.75|theta1
2|T(1,2)|-pi/2|0.35|0|theta2 - pi/2
3|T(2,3)|0|1.25|0|theta3
4|T(3,4)|-pi/2|-0.054|1.5|theta4
5|T(4,5)|pi/2|0|0|theta5
6|T(5,6)|-pi/2|0|0|theta6
G|T(6,G)|0|0|0.303|0

Where:
- `X(i)`, `Y(i)`, `Z(i)` are the 3 axes of frame `i` called `O(i)` and `O(G)` is the gripper frame.
- `T(i-1, 1)`: The homogenous transformation `T` from frame `i-1` to frame `i`.
- `alpha(i-1)`: Twist angle, is the angle between `Z(i-1)` and `Z(i)` along `X(i-1)`.
- `a(i-1)`: Link length, is the distance between `Z(i-1)` and `Z(i)` along `Z(i-1)`.
- `d(i)`: Link offset, is the signed distance from `X(i-1)` to `X(i)` along `Z(i)`.
- `theta(i)`: Joint angle, is the angle between `X(i-1)` and `X(i)` along `Z(i)`.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

I used the `jupyter notebook` to expirement with the transformation matrices derivation. My expirements and step-by-step derivation of the homogeneous transform can be found in the [Kinematics notebook](./Kinematics.ipnyb).

The transform `T(i-1,i)` is the transform of `frame i` called `O(i)` relative to `frame i-1` called `O(i-1)` and is used to transfrom a vector from `O(i)` space to `O(i-1)` space.

Per the modified DH convention, the transform `T(i-1,i)` can be derived as follows:

![Homogeneous transform formula][homo_trans_deriv]
![Homogeneous transform matrix][homo_trans]

When converted to code in the notebook, it can be defined as a function parameterized with `alpha(i-1), a(i-1), theta(i), d(i)` as follows:
```py
def T_i(alpha, a, theta, d):
    T = Matrix([[cos(theta)             , -sin(theta)            ,           0,               a],
                [sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha , -sin(alpha) * d],
                [sin(theta) * sin(alpha), cos(theta) * sin(alpha),  cos(alpha),  cos(alpha) * d],
                [                      0,                       0,           0,               1]])
    return T
```

The homogeneous transform `T(0,N)` is the total transform from `link(0)` aka `base_link` to `link(n)` is defined as follows:

![Total homogeneous transform][homo_trans_T_0_G]

Where `N` in our analysis will be the end-effector or gripper link denoted as `G`. Our goal is to come up with `T(0,G)` i.e. the transfrom between the `base_link` and `gripper link`.

To get familiar with the idea, I first replicated the `Forward Kinematics` sympy code example in Lesson 11, section 17 in the Kinematics notebook to derive the homogeneous transformation matrices, then generalized to the KR210 6DOF to derive `T(0,G)`.

```py
T0_1 = T_i(alpha0, a0, q1, d1)
T0_1 = T0_1.subs(s)

T1_2 = T_i(alpha1, a1, q2, d2)
T1_2 = T1_2.subs(s)

T2_3 = T_i(alpha2, a2, q3, d3)
T2_3 = T2_3.subs(s)

T3_4 = T_i(alpha3, a3, q4, d4)
T3_4 = T3_4.subs(s)

T4_5 = T_i(alpha4, a4, q5, d5)
T4_5 = T4_5.subs(s)

T5_6 = T_i(alpha5, a5, q6, d6)
T5_6 = T5_6.subs(s)

T6_G = T_i(alpha6, a6, qG, dG)
T6_G = T6_G.subs(s)

T0_2 = simplify(T0_1 * T1_2)
T0_3 = simplify(T0_2 * T2_3)
T0_4 = simplify(T0_3 * T3_4)
T0_5 = simplify(T0_4 * T4_5)
T0_6 = simplify(T0_5 * T5_6)
T0_G = simplify(T0_6 * T6_G)
```

One important observation about the KR210 is that our DH parameters frame assignment result in a reference frame `O(G)` different in orientation from that in the URDF description. To make them match, we need to do post multiply T(0,G) by a constant rotation matrix `R_corr` that represents an intrinsic rotation of `180'` about body-fixed Z axis, and then `-90'`about the body-fixed Y axis.

```py
R_z = rot_z(pi)
R_y = rot_y(-pi/2)
R_corr = simplify(R_z * R_y) # Intrinsic rotation about Z and then Y
```
Where `rot_z(...)` defines a rotation matrix about the Z axis, and `rot_z` defines a rotation matrix abou the Y axis.

```py
T_total = simplify(T0_G * R_corr)
```

Where `T_total` is the generalized homogeneous transform from the `base_link` to the `gripper_link` according to the URDF description and is parameterized by the 6 joint angles of the 6 DOF, named from `q1` or `theta1` to `q6` or `theta6`.

To verify the derivation of `T_total`, I evaluated that transformation with the arm in its default position, in which all joint angles are 0.

```py
T_total.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0})
```
The resultant matrix was as follows:

![EE default position transform][arm_default_transform]

Which indicate no change in orientation, but only a transform 2.153 in the X direction and 1.946 in the Z direction relative to the `base_link` frame.

Using the `joint_state_publisher` and setting all the joints in the default 0 angle, and looking into `RVIZ` at the `gripper_link` TF node, we find the numbers match.

![EE default position][ee_default_pos]

The positions in TF are 

The same expirement was repeated with different combination of joint angles using `joint_state_publisher` to verify that the generated `T_total` match.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The last 3 joints 4, 5, 6 of the KR210 constitute a spherical write since the revolute axes intersect in a single point where `joint_5` is the wrist center. We divide the problem into 2 smaller problems:
- The position of the wrist center `WC` i.e `joint_5`. Called Inverse Position Kinematics.
- The orientation of the end effector. Called Inverse Orientaiton Kinematics.

The overall homogeneous transform between the `base_link` and the end-effector (EE) aka the `gripper_link` has the following form:

![Kinematic Decoupling][decoupling_matrix]

The summary of the solution to the inverse kinematics problem is as follows:
- *Step 1:* Find `theta1`, `theta2`, `theta3` such that the wrist center has coordinates given by
![Wrist Center Formula][wrist_center_formula]
- *Step 2:*: Using the joint variables determined in Step 1, evaluate `R(0,3)`
- *Step 3:*: Find a set of Euler angles corresponding to the rotation matrix
![R3_6 Rotation Matrix][R3_6_matrix_derivation]


##### 1. Inverse Position Kinematics Problem
|||
--|--|--
*Description:*|Given the `gripper_link` pose, find out the wrist center `WC` position, and use that to compute the angle of the first 3 joints.
*Input:*|EE position `(px, py, pz)`, and EE orientation `(roll, pitch, yaw)`
*Output:*|`theta1`, `theta2`, and `theta3`

*Solution*


##### 2. Inverse Orientation Kinematics Problem
|||
--|--|--
*Description:*|Given the first three joint angles, find the final three joint angles corresponding to a given orientation `R(3,6)` with respect to frame `O(3)`.
*Input:*|`theta1`, `theta2`, and `theta3`
*Output:*|`theta4`, `theta5`, `theta6`

*Solution*

We can observe that the matrix `R(0,6)` can be decomposed as follows:
```
R(0,6) = R = R(0,3) * R(3,6)
```
and the orientation of the end effector relative to `O(3)` can be expressed as
```
R(3,6) = inv(R(0,3)) * R
```
where R, the orientation of the EE relative to the `base_link` is given as input to the problem by the simulation.

R(0,3) can be trivially computed by extracting the rotation part of the homogeneous transform `T(0,3)` and evaluating it using the known first 3 joint variables which can be expressed as follows:

![R0_3_matrix]

The next step is to compute `R(3,6)` by evaluating `inv(R(0,3)) * R` from which we can workout the last 3 angles using trigonometry rules.

![R3_6_matrix]

From `R(3,6)` above, we can index the elements as `rij` for row `i` and column `j` assuming a 1-based indexing. We use atan2 to avoid quadrant ambiguity.

```py
theta6 = atan2( r21, r11)                    # alpha, rotation about z-axis
theta5 = atan2(-r31, sqrt(r11**2 + r21**2))  # beta,  rotation about y-axis
theta4 = atan2( r32, r3;3)                    # gamma, rotation about x-axis
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


