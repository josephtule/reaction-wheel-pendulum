# Control of a Pendulum via Reaction Wheel

## Motivation
This repo will document my entire process of designing and implementing a controller to stabilize a reaction wheel in the vertical position.  
The main purpose is to familiarize myself with the design process, reinforcing my knowledge of dynamics and control, and learn new skills along the way (Arduino, C++, CAD, etc.).

The "up" position of a pendulum is an inherently unstable position, in order to keep it in this state, external control must be used. To do this, a reaction wheel will be used where the act of accelerating the wheel will apply a torque to the system which will serve as the input for control.


## Equations of Motion
To start designing a controller, a model should be constructed for the system. It should be noted that this model of the system will be fairly rough, as in there will be assumptions applied to the system in order to create easily derivable dynamics.

### Assumptions:
- When calculating the inertia tensor for the system, the pendulum and wheel system will be assumed to be the combination of a rod and a point mass about the origin point.
- The 

The first step is to draw the reference frames that will be used throughout the process. The first coordinate frame will be the "intertial" frame fixed at the origin the the $\hat{i}$ direction point to the right (horizontally) with the $\hat{j}$ direction pointing up 90 degrees from the horizontal plane, the third axis will the $\hat{k} = \hat{i} \times \hat{j}$ using the right hand rule to get a vector that is perpendicular to the first two vectors.

The second set of of vectors that will be used will be defined as $\hat{b_1}$ pointing in the direction of pendulum arm, $\hat{b_2}$ 90 degrees rotated counter-clockwise in the plane of the pendulum, and $\hat{b_3} = \hat{b_1} \times \hat{b_3}$.

Similarly the last set of vectors will be $\hat{c_1}$ pointing towards an arbitrary point, Q, on the reaction wheel. $\hat{c_2}$ 90 degrees counter-clockwise of the previous vector, and $\hat{c_3} = \hat{c_1} \times \hat{c_2}$.

Note, $\hat{k}$, $\hat{b_3}$, and $\hat{c_3}$ are all parallel.

Once the reference frames are set up, the position vectors for points of interest can be created. These position vectors will be with respect to the origin point, O. 

$$\vec{r}_{OP} = \ell \hat{b_1}$$ 

and

$$\vec{r}_{OQ} = \vec{r}_{OP} + r \hat{c_1} = \ell \hat{b_1} + r \hat{c_1}$$

In this problem, we're interested in the angles of 1) the pendulum, and 2) the wheel, as well as their first and second derivatives. This actually makes the previous section irrelevant (the position vectors) but I will keep them there because they look nice. 

First, we will look at the reaction wheel/motor assembly as a first subsystem. Torque is generated when the reaction wheel is accelerated. The relationship between torque and the angular acceleration is as follows:

$$\vec{\tau} = I\vec{\alpha}$$

where $\tau$ is the torque vector, I is the inertia tensor, and $\alpha$ is the angular acceleration vector. In our case, the relation ship is as follows:

$$\vec{\tau_W} = I_{W}\vec{\alpha{W}}$$

Where the subscript W denotes a property of the reaction wheel.

To get $\vec{\alpha}}$, take the derivative of the angle of an arbitrary point on the wheel, Q. Usually, if the axis of rotation didn't match an direction vector in the inertial frame, taking the derivative would require kinematic decomposition (basic kinematic equation, BKE) where

$$^{i}\frac{d}{dt}  ^{b}(\cdot) =  ^{b} \frac{d}{dt}(\cdot) + ^{i}\omega^{b} \times  ^{b}(\cdot)$$

where an i superscript denotes a derivative or object with respect to (wrt) the inertial frame, a b superscript denotes a derivative or object wrt the body frame, and $^{i}\omega^{b}$ is the angular velocity of the body frame in the inertial frame.

This is not required in our case because the angular velocities are lined up with an axis of the inertial frame which makes the cross product term = 0.


## Control Laws

### Challenges

## 3D Printing / CAD

## Components

## Arduino

## MATLAB Code

## Arduino Code
