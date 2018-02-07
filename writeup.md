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
[WC_deriv]: ./misc_images/WC_deriv.png
[WC2_deriv]: ./misc_images/WC2_deriv.png
[DH_schematics]: ./misc_images/DH_schematics.png
[theta1_exp]: ./misc_images/theta1_exp.png
[phi1234_exp]: ./misc_images/phi1234_exp.png
[SSS_exp]: ./misc_images/SSS_exp.png
[theta23_exp]: ./misc_images/theta23_exp.png
[theta456_exp]: ./misc_images/theta456_exp.png
[R3_6_exp]: ./misc_images/R3_6_exp.png
[KR210_IK]: ./misc_images/KR210_IK.png
[T0_1]: ./misc_images/T0_1.png
[T1_2]: ./misc_images/T1_2.png
[T2_3]: ./misc_images/T2_3.png
[T3_4]: ./misc_images/T3_4.png
[T4_5]: ./misc_images/T4_5.png
[T5_6]: ./misc_images/T5_6.png
[T6_G]: ./misc_images/T6_G.png
[T0_G_simplified]: ./misc_images/T0_G_simplified.png
[T_total]: ./misc_images/T_total.png
[rviz_drop]: ./misc_images/rviz_drop.png
[gazebo_grap]: ./misc_images/gazebo_grap.png
[gazebo_drop]: ./misc_images/gazebo_drop.png
[gazebo_drop2]: ./misc_images/gazebo_drop2.png
[gazebo_stacked]: ./misc_images/gazebo_stacked.png

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

Then we draw* schematics for the KR210 6DOF manipulator based on the modified DH convention and the DH parameter assignment algorithm mentioned in the lectures:

![DH Schematics][DH_schematics]

*Note: The schematics drawing from the lecture video was used here as is for time constraints.

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

I used the `jupyter notebook` to expirement with the transformation matrices derivation. My expirements and step-by-step derivation of the homogeneous transform can be found in the [Kinematics notebook](./Kinematics.ipynb).

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
Where `rot_z(..)` defines a rotation matrix about the Z axis, and `rot_Y(..)` defines a rotation matrix about the Y axis. R_corr can be denoted as `R(DH,URDF)` or the `URDF` frame relative to `DH` frame, and it simply transforms from the `URDF` or ROS convetion to the `DH` convetion when pre-multiplied with a vector expressed in the `URDF` frame, that's why it is post-multiplied with `T(0,G)` and hence the last rotation matrix in the matrix chain multiplicaiton resulting in `T_total` as follows:

```py
T_total = simplify(T0_G * R_corr)
```

Where `T_total` is the generalized homogeneous transform from the `base_link` to the `gripper_link` according to the URDF description and is parameterized by the 6 joint angles of the 6 DOF, named from `q1` or `theta1` to `q6` or `theta6`.

Following is the individual homogeneous transforms as generated by the notebook:

![][T0_1]

![][T1_2]

![][T2_3]

![][T3_4]

![][T4_5]

![][T5_6]

![][T6_G]

`T_total` below `T0_G` after multiply by R_corr and applying `simplify`:

![][T_total]

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
*Description*|Given the `gripper_link` pose, find out the wrist center `WC` position, and use that to compute the angle of the first 3 joints.
--|--
*Input*|EE position `(px, py, pz)`, and EE orientation `(roll, pitch, yaw)`
*Output*| Wrist Center `WC` position, and hence the `theta1`, `theta2`, and `theta3`

*Solution*

Since `joint_5` represents the wrist center denoted `WC`, we want to find its position relative to the `base_link` following the guidelines presented in `15. Inverse Kinematics with Kuka KR210`.

Following is the derivation of the wrist center `WC` position relative to `base_link`:

![WC Derivation][WC_deriv]

Where:
- EE is the end-effector (`gripper_link`) position relative to the `base_link` and is given.
- R_rpy is the `gripper_link` x-y-z extrinsic rotation relative to the `base_link` and it gets post multiplied by R_corr to transform the EE axes from URDF to DH convention.

![][theta1_exp]

We then transform `WC` from `base_link` frame to `link_2` frame named `O(2)`

![][WC2_deriv]

Once we have WC relative to `O(2)`, we can draw the rest of the manipulator starting from `joint_2` and till `WC`:

![][KR210_IK]

Then we derive the formulas for the lenght of the 3 sides of our SSS triangle named `A, B, C`. We find that both A and C are constants and can be pre-computed from the DH table, while `B` depends only on the variable `WC`.

![][SSS_exp]

![][phi1234_exp]

![][theta23_exp]

##### 2. Inverse Orientation Kinematics Problem
*Description*|Given the first three joint angles, find the final three joint angles corresponding to a given orientation `R(3,6)` with respect to frame `O(3)`.
--|--|
*Input*|`theta1`, `theta2`, and `theta3`
*Output*|`theta4`, `theta5`, `theta6`

*Solution*

We can observe that the matrix `R(0,6)` can be decomposed as follows:
```
R(0,6) = R_rpy = R(0,3) * R(3,6)
```
and the orientation of the end effector relative to `O(3)` can be expressed as
```
R(3,6) = inv(R(0,3)) * R
```
where R, the orientation of the EE relative to the `base_link` is given as input to the problem by the simulation.

R(0,3) can be trivially computed by extracting the rotation part of the homogeneous transform `T(0,3)` and evaluating it using the known first 3 joint variables.

The `R(3,6)` rotation matrix can be evaluated in its symbol form using the notebook.

```py
R0_3 = T0_3[0:3,0:3]
R0_3
```

![R0_3_matrix]

The next step is to compute `R(3,6)` by evaluating `inv(R(0,3)) * R` from which we can workout the last 3 angles using trigonometry rules.

The symbol form of `R(3,6)` can be figured out by using the notebook.

```py
R3_6 = simplify(T3_4 * T4_5 * T5_6)[:3,:3]
R3_6
```

![R3_6_matrix]

From `R(3,6)` above, we can index the elements as `rij` for row `i` and column `j` assuming a 1-based indexing.

![][R3_6_exp]

From `R(3,6)` in its symbol form and with the use of trigonometry, pythagorean identities and especially `atan2` - to avoid quadrant ambiguity - we can derive `theta5` and then `theta4` and `theta6` as piecewise functions follows.

![][theta456_exp]


We see that `theta5` has 2 `atan2` solutions and they are based on the postive and negative solutions of the square root i.e `atan2(-y,x)` and `atan2(y,x)`. The 2 soltuions resembles the wrist flip up and flop down configurations. For each `theta5` solution there is a corresponding `theta4` and `theta6`. 

#### General Case `sin(theta5) != 0`
When `sin(theta5)` is non-zero, there exist a solution for both `theta4` and `theta6` as based on `R3_6`:

```
Let sin(q5) < 0
Let r33 =  sin(q4)sin(q5)
Let r13 = -cos(q4)sin(q5)
Then,
q4 = atan2(-r33, r31)
q4 = atan2(-sin(q4)sin(q5), -cos(q4)sin(q5))
q4 = atan2(sin(q4), cos(q4))

Let sin(q5) < 0
Let r22 = -sin(q5)sin(q6)
Let r21 =  sin(q5)cos(q6)
Then,
q6 = atan2(r22, -r21)
q6 = atan2(-sin(q5)sin(q6), -sin(q5)cos(q6))
q6 = atan2(sin(q6), cos(q6))

Let sin(q5) > 0
Let r33 =  sin(q4)sin(q5)
Let r13 = -cos(q4)sin(q5)
Then,
q4 = atan2(r33, -r13)
q4 = atan2(sin(q4)sin(q5), cos(q4)sin(q5))
q4 = atan2(sin(q4), cos(q4))

Let sin(q5) > 0
Let r22 = -sin(q5)sin(q6)
Let r21 =  sin(q5)cos(q6)
Then,
q6 = atan2(-r22, r21)
q6 = atan2(sin(q5)sin(q6), sin(q5)cos(q6))
q6 = atan2(sin(q6),cos(q6))
```

#### Special Case `sin(theta5) == 0` (Singularity)
If `sin(theta5)` is zero, then R3_6 matrix will have mostly zero elements and we can only compute `theta4` or `theta6`. In our implementation we choose to fix `theta4` to zero and solve `theta6` since it can be isolated wyeh `sin(theta4)` and `sin(theta5)` are zeros as shown up in our piecewise function definition for `theta4` and `theta6`. This is the special case of singularity and happens when `link_4` and `link_6` are collinear.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The IK solver service is divided into 2 modules

#### `kr210_solver.py`
This module exposes 2 approaches and hence implementations to the IK and FK problem, and is written to be extendible with more approaches. Each implementation should be implemented as a class that exposes 2 methods:
- `solve_IK`: Solves the Inverse Kinematics problem given the end-effector pose relative to the `base_link`. The solution is the 6 joint angles.
- `solve_FK`: Solves the Forward Kinematics problem given the 6 joint angles. The solution is the end-effector pose relative to the `base_link`.

The module pre-computes the following for reuse as read-only global variables for all IK and FK solvers:
- Various DH `sympy` symbols: `alpha0-6, a0-6, q0-6, d0-6, qG, dG, r, p, y, px, py, pz`.
- DH params table based on the modified DH convention.
- A correctional rotation matrix `R_corr` used to accomodate for the difference between URDF and our DH frame assignment for `gripper_link`.
- Homogeneous transforms from `frame i-1` to `frame i` named Ti-1_i and most importantly T0_G, the transfrom from frame0 the `base_link` to last frame `gripper_link`.
- `R0_3` rotation matrix to be re-used in IK inverse rotation step.

The following are the 2 provided implementations:
- `Kr210Solver` is the base and hence default implementation for solving the IK and FK problem by reusing all the pre-computed matrices, it implements the methods:
    - `solve_IK`: following the outlined algorithm in the lectures step-by-step, it figures out `WC`, computes `q1` and use trigonemetry to compute `q2` and `q3` as part of the Inverse Position Kinematics, it then uses `q1-3` to evaluate `R0_3` and solve the Inverse Orientation Kinematics problem i.e `q4, q5, q6`.
    - `solve_FK`: as simple as T0_G.evalf with the input joint variables q0-6 and the px,py,pz are extracted and returned from the resulting matrix.
- `Kr210LazySolver` is an experimental implementation which tries to utilize the powerful expressive power of `sympy` by expressing each joint variable as a closed formula in terms of the EE pose for `q1-3` and EE pose and `q1-3` valus for `q4-6`.
    - `Constructor`: follows the same algorithm outlined in the lecture but it doesn't evaluate the symbols in the intermediate steps, instead it solves both the Inverse Position and Orientation Kinematics symbolically by forming the closed form expression for each joint variable `q1-6`.
    - `solve_IK`: the first 3 joint variables `q1-3` are evaluated by calling evalf and subs with EE pose. The oter 3 joint variables `q4-6` are evaluated by calling evalf and subs with both `q1-3` and EE pose.
    - `solve_FK`: as simple as T0_G.evalf with the input joint variables q0-6 and the px,py,pz are extracted and returned from the resulting matrix.

`Kr210SolverFactory`: implements a static factory method for creating instances of the KR210 solvers based on the passed solver name. In this implementation the class name string is the implementation identifier. It exposes a single `create` method for handling creation.

#### `IK_server.py`
Implements the ROS IK_server service by delegating call to the instantiated KR210 solver.

Here is how `Kr210SolverFactory` is used to create the KR210 base implementation.
```py
# Use KR210 solver factory method to instantiate a single IK solver to use
# throught the IK_server service lifetime.
solver = Kr210SolverFactory.create("Kr210Solver")
```

A global boolean flag is provided to aid in debugging and solver output verificaiton for both IK and FK.

```py
# If set to True, the output of the IK step will be used to compute FK and the
# the error will be computed and logged to console, otherwise set to False.
debug_IK = True
```
Here is how FK is compared against IK and the error is recorded.

```py
# Compute error between FK and IK and append to list
ee_x_e = abs(fk_px - px)
ee_y_e = abs(fk_py - py)
ee_z_e = abs(fk_pz - pz)
ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
ee_offset_list.append(ee_offset)
```

### Results and Areas of Improvements

Following is a screenshot of pick and place from location 9:

![][rviz_drop]
![][gazebo_grap]
![][gazebo_drop]

Another attempt for pick and place:

![][gazebo_drop2]

The following screenshot shows the arm completing 3 cycles of pick and place where the 3 items are stacked over each other as an indication of arm consistency:

![][gazebo_stacked]

The forward and inverse kinematics implementation went through 3 incremental phases of development and verification:
- `Jupyter notebook`: goes through the IK algorithm outlined in lectures in small increments, most steps are first written in TeX and then implemented in python underneath.
- `IK_debug.py`: a test module that runs in isolation from ROS simulation, and tests `kr210_solver.py` against a set of pre-defined test cases.
- `IK_server.py`: the ROS simulation run-time module that handles IK_server service requests using `kr210_solver.py`.

#### Error Margin
`IK_server.py` has the capability to do verification of the IK solution by comparing the FK solution using the output from IK against the ROS requested EE pose. It was found that FK and IK for `Kr210Solver` and ``Kr210LazySolver` always match with a margin of 0 error.

```
[INFO] [1517688124.428036, 1723.618000]: EE offset: avg 0.00000000 min 0.00000000 max 0.00000000 units

```

### Unnecessary wrist rotations
Following is a sample console output snippet of the last 3 joint angles in degrees for a generated trajectory:
```
[INFO] [1518040152.795900, 1376.060000]: -177.65042385' 52.87180837' -30.08771899'
[INFO] [1518040152.796086, 1376.060000]: -177.98413421' 53.73516876' -29.30744595'
[INFO] [1518040152.796293, 1376.060000]: -178.77104243' 55.46092116' -27.78139275'
[INFO] [1518040152.796577, 1376.061000]: -179.52467841' 57.22455497' -26.28011064'
[INFO] [1518040152.796765, 1376.061000]: 178.90292864' 60.84736046' -23.38176031'
[INFO] [1518040152.796950, 1376.061000]: 177.43382063' 64.62260279' -20.60916793'
[INFO] [1518040152.797133, 1376.061000]: 175.67337614' 69.52520925' -17.34919765'
[INFO] [1518040152.797432, 1376.062000]: 174.01461627' 74.58520567' -14.33010020'
```
You can notice that `joint4` switches from `-179'` to `178'` instantly, and when `MoveIt!` executes this trajectory and reaches the transition from `-179'` to `178'`, it reverts the joint back to angle 0 in the negative direction `-179'`, and then starts another `178'` in the positive direction, while the perfect movement would have been to continue move an extra `-3'` to reach the same orientation to reach `-182` which is still in the joint limit. This problem happens because we use `atan2` which restricts the output to be in the range `-pi` to `pi`. The usage of `atan2` results in an effecient use of the joints capabilities. An improvement would be to implement a logic to complement `atan2` results to produce shorter and effecient movements.

#### Solvers performance
The `Kr210Solver` is able to do IK for each pose in avarge of 0.04174614s (~ 42ms) thanks to the pre-computations that happen on node startup.

```
[INFO] [1517687659.466173]: Received 37 eef-poses from the plan
[INFO] [1517687661.135972]: length of Joint Trajectory List: 37, total time: 1.6696
[INFO] [1517687661.136172]: Solve time avg 0.04174614s min 0.07964897s max 0.04504955s
```
As we see from IK_server.py console output, a 37 pose takes total of 1.6 seconds which may not be fast enough in production systems.

The experimental `Kr210LazySolver` on the other hand is much slower but still able to solve the problems correctly. Its average IK solve time is 0.77 seconds (~800ms) and that is 20 times slower than the base implementation.

```
[INFO] [1517688718.511352, 2310.964000]: Received 35 eef-poses from the plan
[INFO] [1517688747.791727, 2338.801000]: length of Joint Trajectory List: 35, total time: 29.2801
[INFO] [1517688747.792055, 2338.801000]: Solve time avg 0.77662206s min 1.07123685s max 0.83649506s
```
I tried to simplify each theta closed-form in the class constructor but the node never started, the `simplify` helper takes too much time and then throws exceptions internal to `sympy` and figuring out the problem is beyond my interests.

#### Arm movements
It is very noticable that the arm moves very slowely once the trajectory is computed, which I am not able to explain yet. `MoveIt!` doesn't actually execute the arm movement until the arm trajectory is fully computed and hence slowness is unlikely to result from IK solve time. This is an area of investigation. We need to figure out whether this is a simulation issue or not since the performance is unacceptable in produciton environment.

#### Choosing betwee multiple solutions
It is worth noting that in the current implementation, we choose always the first solution of the tuble `(theta4, theta5, theta6)` in `Kr210Solver.choose_best_q4q5q6` without considering the current state of the joints, some improvement can be done here to track the joint state and always choose the solution that is closest to the previous trajectory to avoid unnecessary joint rotations.
