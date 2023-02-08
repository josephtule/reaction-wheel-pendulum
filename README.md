# Control of a Pendulum via Reaction Wheel

## Motivation
This repo will document my entire process of designing and implementing a controller to stabilize a reaction wheel in the vertical position.  
The main purpose is to familiarize myself with the design process, reinforcing my knowledge of dynamics and control, and learn new skills along the way (Arduino, C++, CAD, etc.).

The "up" position of a pendulum is an inherently unstable position, in order to keep it in this state, external control must be used. To do this, a reaction wheel will be used where the act of accelerating the wheel will apply a torque to the system which will serve as the input for control.


## Equations of Motion
To start designing a controller, a model should be constructed for the system. It should be noted that this model of the system will be fairly rough, as in there will be assumptions applied to the system in order to create easily derivable dynamics.

### Assumptions:
- When calculating the inertia tensor for the system, the pendulum and wheel system will be assumed to be the combination of a rod and a point mass about the origin point.
- The reaction wheel will be modeled as a cylindrical tube with a uniform mass.

### Reference Frames
The first step is to draw the reference frames that will be used throughout the process. The first coordinate frame will be the "intertial" frame fixed at the origin the the $\hat{i}$ direction point to the right (horizontally) with the $\hat{j}$ direction pointing up 90 degrees from the horizontal plane, the third axis will the $\hat{k} = \hat{i} \times \hat{j}$ using the right hand rule to get a vector that is perpendicular to the first two vectors.

The second set of of vectors that will be used will be defined as $\hat{b_1}$ pointing in the direction of pendulum arm, $\hat{b_2}$ 90 degrees rotated counter-clockwise in the plane of the pendulum, and $\hat{b_3} = \hat{b_1} \times \hat{b_3}$.

Similarly the last set of vectors will be $\hat{c_1}$ pointing towards an arbitrary point, Q, on the reaction wheel. $\hat{c_2}$ 90 degrees counter-clockwise of the previous vector, and $\hat{c_3} = \hat{c_1} \times \hat{c_2}$.

Note, $\hat{k}$, $\hat{b_3}$, and $\hat{c_3}$ are all parallel.

From these reference frames as well as knowing the angles between them, rotation matrices can be created to help switch between the frames. This may or may not be used later on, but it is always good to have in your back pocket.

Once the reference frames are set up, the position vectors for points of interest can be created. These position vectors will be with respect to the origin point, O. 

$$\vec{r}_{OP} = L \hat{b_1}$$,

$$\vec{r}_{CM} = \ell \hat{b_1}$$ 

and

$$\vec{r}_{OQ} = \vec{r}_{OP} + r \hat{c_1} = L \hat{b_1} + r \hat{c_1}$$

In this problem, we're interested in the angles of 1) the pendulum, and 2) the wheel, as well as their first and second derivatives. This actually makes the previous section irrelevant (the position vectors) but I will keep them there because they look nice. 

### Reaction Wheel

First, we will look at the reaction wheel/motor assembly as a first subsystem. Torque is generated when the reaction wheel is accelerated. The relationship between torque and the angular acceleration is as follows:

$$\vec{\tau} = I\vec{\alpha}$$

where $\tau$ is the torque vector, I is the inertia tensor, and $\alpha$ is the angular acceleration vector. In our case, the relation ship is as follows:

$$\vec{\tau_{w}} = I_{w}\vec{\alpha_{w}}$$

Where the subscript W denotes a property of the reaction wheel and the inertia tensor with respect to the center of the wheel and will be a function of the mass of the wheel but not the motor because the wheel in spinning in this case and the motor is considered stationary in the c-frame.

To get $\vec{\alpha}$, take the derivative of the angle of an arbitrary point on the wheel, Q. Usually, if the axis of rotation didn't match an direction vector in the inertial frame, taking the derivative would require kinematic decomposition (basic kinematic equation, BKE) where

$$^{i}\frac{d}{dt}  ^{b}(\cdot) =  ^{b} \frac{d}{dt}(\cdot) + ^{i}\omega^{b} \times  ^{b}(\cdot)$$

where an i superscript denotes a derivative or object with respect to (wrt) the inertial frame, a b superscript denotes a derivative or object wrt the body frame, and $^{i}\omega^{b}$ is the angular velocity of the body frame in the inertial frame.

This is not required in our case because the angular velocities are lined up with an axis of the inertial frame which makes the cross product term = 0. Taking this derivative gives us:

$$\vec{\alpha_{W}} = \ddot{\theta} \hat{c_3} = \ddot{\theta} \hat{k}$$

which gives us:

$$\vec{\tau_{w}} = I_{w}\ddot{\theta} \hat{k}$$

### Pendulum System

The torque generated by the reaction wheel can be seen as an applied torque in the opposite direction (Newton's law of action-reaction) when the system consists of the pendulum. Looking at the figure, it can be seen that the forces, or moments (torques) in our case, are the moment due to gravity as well as the torque supplied by the reaction wheel. Using the torque relationship presented before:

$$\Sigma \vec{\tau} = \vec{\tau_{w}} + \vec{\tau_{g}} = I_w\ddot{\theta} (-\hat{k}) + \vec{\tau_{g}}$$
$$ = I_{p}\ddot{\phi}$$

Here, $\ddot{\phi}$ is derived in the same manner as $\ddot{\theta}$. The next problem will be determining to torque due to gravity on the pendulum.

It is known that torque due to gravity is equivalent to a moment arm with a force acting on the center of mass. Where the torque equation is given as:

$$\vec{\tau} = \vec{r}_{CM} \times \vec{F}$$

which turns into 

$$\vec{\tau}_{g} = \ell \hat{b_1} \times m_p g (-\hat{j})$$

Looks like we did use one of the position vectors from above. We will also use one of the rotation matrices (or a portion of it) determined above where:

$$\hat{j} = cos(\phi) \hat{b_1} - sin(\phi) \hat{b_2}$$

which changes our gravity torque

$$\vec{\tau}_{g} = \ell \hat{b_1} \times m_p g (-(cos(\phi) \hat{b_1} - sin(\phi) \hat{b_2}))$$

$$ = \ell m_p g sin(\phi) \hat{b_3} = \ell m_p g sin(\phi) \hat{k}$$

Where $m_p$ is the mass of the pendulum arm, the motor, and the reaction wheel. Substituting this back into our pendulum torque gives us 

$$I_w\ddot{\theta} (-\hat{k}) + \vec{\tau_{g}} = -I_w\ddot{\theta} \hat{k} + \ell m_p g sin(\phi) \hat{k}$$

$$ = I_{p}\ddot{\phi}$$

This gives us the equations of motion for the system. Here we can determine if we want to include $\ddot{\theta}$ as a state variable or as an input. To decrease the system order, we will use it as an input which will give is a state vector in $\mathbb{R}^2$.

One last consideration is the friction about the rotation point, this can be added on as follows

$$ I_{p}\ddot{\phi} = -I_w\ddot{\theta} \hat{k} + \ell m_p g sin(\phi) \hat{k} + \mu \ell \dot{\phi} \hat{k}$$

Where $\mu$ is the coefficient of friction, since we are using a smooth bearing, this value will be very small. This uses the method shown in this video [Friction Pendulum Video](https://www.youtube.com/watch?v=SZWn7x4g-Vo)
### State-Space

We will use various state-space methods to generate a control scheme for the reaction wheel, so conversion into state-space is a logical next step. To do so, we will take the state vector as

$$\vec{x} = \left\lbrack \begin{array}{c}
x_{1} \\
x_{2}
\end{array}\right\rbrack$$

$$ = \left\lbrack \begin{array}{c}
\theta \\
\dot{\theta} 
\end{array}\right\rbrack $$

then taking the derivative

$$\dot{\vec{x}} = \left\lbrack \begin{array}{c}
\dot{\theta} \\
\ddot{\theta} 
\end{array}\right\rbrack = \left\lbrack \begin{array}{c}
\dot{x_{1}} \\
\dot{x_{2}}
\end{array}\right\rbrack$$ 

$$ = \left\lbrack \begin{array}{c}
\dot{x_{2}} \\
I_{p}^{-1} (m_m g \ell sin(x_{1}) - \mu \ell x_{2} - I_{w} u) \hat{k}
\end{array}\right\rbrack$$

Which is the state-space form of your equations of motion.

### Moments of Inertia

The last part of the mathematical model is to determine the various inertia tensors that were used in the EOMs, these can easily be found using Google. This includes the inertia tensor for a cylindrical tube (reaction wheel)

$$ I_{w} = \left\lbrack \begin{array}{ccc}
\frac{1}{12}m_w \left(3\left(r_2^2 +r_1^2 \right)+h^2 \right) & 0 & 0\\
0 & \frac{1}{12}m_w \left(3\left(r_2^2 +r_1^2 \right)+h^2 \right) & 0\\
0 & 0 & \frac{1}{3}m_w \left(r_2^2 +r_1^2 \right)
\end{array}\right\rbrack$$

The next inertia tensor is for the pendulum assembly about the origin point, O. As mentioned before, this will be modeled as a rod with a point mass attatched to the opposite end. The inertia tensor for a rod with a point mass at the end about the point O is

$$I_p = \left\lbrack \begin{array}{ccc}
m_{\mathrm{rod}} L^2  & 0 & 0\\
0 & \frac{1}{3}m_{\mathrm{rod}} L^2 & 0\\
0 & 0 & \frac{1}{3}m_{\mathrm{rod}} L^2
\end{array}\right\rbrack +\left\lbrack \begin{array}{ccc}
0 & 0 & 0\\
0 & 0 & 0\\
0 & 0 & m_w L^2 
\end{array}\right\rbrack$$

$$ = \left\lbrack \begin{array}{ccc}
m_{\mathrm{rod}} L^2  & 0 & 0\\
0 & \frac{1}{3}m_{\mathrm{rod}} L^2 & 0\\
0 & 0 & \frac{1}{3}m_{\mathrm{rod}} L^2 + m_{w} L^2
\end{array}\right\rbrack$$

## Control
### Challenges

One of the challenges of controlling this system is the saturation of the motor. Since motors have a maximum RPM, the motors can't accelerate higher than this limit to apply additional torque. Traditional control methods such as pole placement and LQR are unable to set contraints on the maximum RPM of the motors into account and will be seen when calculating the gains for those methods and simulating the system. A numerical method called model predictive control (in linear and nonlinear flavors) can be used alongside constraints in order to create a control scheme for the reaction wheel. The only problem is the intensive computation required to apply MPCs online.

### Linearization


### Pole Placement
One of the simplest methods of controlling a state-space system is to apply a pole placement. This control method takes

$$\vec{u}(t) = -K \vec{x}(t)$$ 

Where K is a row vector with the same length as B is tall and will be the gains for the system (the number of states in the system). Assuming the system state-space dynamics have been linearized in the form

$$\dot{\vec{x}}(t) = A \vec{x}(t) + B \vec{u}(t)$$

The input can be substituted in resulting in

$$\dot{\vec{x}}(t) = A \vec{x}(t) + B -(K \vec{x}(t)) = (A - BK)\vec{x}(t)$$

To make the system stable, the eigenvalues of the new state-space system have to be on the open left hand plane of the complex plane. This means that the real parts of the eigenvalues have to be strictly negative in order for the system to "decay" towards a critical point (the equilibrium point found in linearizing the system). To do so, determine the characteristic equation of the new system matrix $(A - BK)$ where the values of K are underdetermined 

$$ \chi_{A}(s) = |\lambda I - A + BK| $$

Pole placement is the act choosing the poles or eigenvalues desired for a system. Choosing stable poles causes a system to be stable. Once the stable poles are chosen (2 needed for our system), create a desired characterisitc equation and set it equal to the system characteristic equation

$$ \chi_{d}(s) = (\lambda_1)(\lambda_2)...(\lambda_n) = |\lambda I - A + BK| $$

Expand the set of binomials of the left hand side and compute the determinant of the right hand side then solve for the undetermined elements in the K matrix. The numerical results can be seen below in the MATLAB results section.


### LQR
### MPC

## Estimation
## Extended Kalman Filter

## 3D Printing / CAD

## Components
### Arduino
### Motors
### Sensors

## Build

## Parameters
Pendulum Arm Mass:

Pendulum Length:

Wheel Mass:

Motor Mass

Center of Mass Length:

Arm + Motor + Wheel Inertia:

Wheel Inertia:



## MATLAB Code

## Arduino Code
