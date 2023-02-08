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

Similarly the last set of vectors will be $\hat{c_1}$ pointing towards an arbitrary point, q, on the reaction wheel. $\hat{c_2}$ 90 degrees counter-clockwise of the previous vector, and $\hat{c_3} = \hat{c_1} \times \hat{c_2}$.



## Control Laws

### Challenges

## 3D Printing / CAD

## Components

## Arduino

## MATLAB Code

## Arduino Code
