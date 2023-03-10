# Control of a Pendulum via Reaction Wheel

## Motivation
This repo will document my entire process of designing and implementing a controller to stabilize a reaction wheel in the vertical position.  
The main purpose is to familiarize myself with the design process, reinforcing my knowledge of dynamics and control, and learn new skills along the way (Arduino, C++, CAD, etc.).

The "up" position of a pendulum is an inherently unstable position, in order to keep it in this state, external control must be used. To do this, a reaction wheel will be used where the act of accelerating the wheel will apply a torque to the system which will serve as the input for control.


## Equations of Motion
To start designing a controller, a model should be constructed for the system. It should be noted that this model of the system will be fairly rough, as in there will be assumptions applied to the system in order to create easily derivable dynamics.

### Assumptions
- When calculating the inertia tensor for the system, the pendulum and wheel system will be assumed to be the combination of a rod and a point mass about the origin point.
- The reaction wheel will be modeled as a cylindrical tube with a uniform mass.
- There will be other assumptions that will be brought up later as they come up in the modelling process.

### Reference Frames
The first step is to draw the reference frames that will be used throughout the process. The first coordinate frame will be the "intertial" frame fixed at the origin the the $\hat{i}$ direction point to the right (horizontally) with the $\hat{j}$ direction pointing up 90 degrees from the horizontal plane, the third axis will be $\hat{k} = \hat{i} \times \hat{j}$ using the right hand rule to get a vector that is perpendicular to the first two vectors.

The second set of of vectors that will be used will be defined as $\hat{b}_1$ pointing in the direction of pendulum arm, $\hat{b}_2$ 90 degrees rotated counter-clockwise in the plane of the pendulum, and $\hat{b}_3 = \hat{b}_1 \times \hat{b}_2$.

Similarly the last set of vectors will be $\hat{c}_1$ pointing towards an arbitrary point, Q, on the reaction wheel. $\hat{c}_2$ 90 degrees counter-clockwise of the previous vector, and $\hat{c}_3 = \hat{c}_1 \times \hat{c}_2$.

Note, $\hat{k}$, $\hat{b}_3$, and $\hat{c}_3$ are all parallel.

From these reference frames as well as knowing the angles between them, rotation matrices can be created to help switch between the frames. This may or may not be used later on, but it is always good to have in your back pocket.

Once the reference frames are set up, the position vectors for points of interest can be created. These position vectors will be with respect to the origin point, O. 

$$\vec{r}_{OP} = L \hat{b}_1$$,

$$\vec{r}_{CM} = \ell \hat{b}_1$$ 

and

$$\vec{r}_{OQ} = \vec{r}_{OP} + r \hat{c}_1 = L \hat{b}_1 + r \hat{c}_1$$

In this problem, we're interested in the angles of 1) the pendulum, and 2) the wheel, as well as their first and second derivatives. This actually makes the previous section irrelevant (the position vectors) but I will keep them there because they look nice. 

### Reaction Wheel

First, we will look at the reaction wheel/motor assembly as a first subsystem. Torque is generated when the reaction wheel is accelerated. The relationship between torque and the angular acceleration is as follows:

$$\vec{\tau} = I\vec{\alpha} $$

where $\tau$ is the torque vector and I is the inertia tensor, and $\alpha$ is the angular acceleration vector. In our case, the relation ship is as follows

$$\vec{\tau}_w = I_{w}\vec{\alpha}_w = \frac{K_{\tau}}{R_{i}} V - \frac{K_{\tau}^2}{R_{i}} \dot{\theta} $$

Where the subscript W denotes a property of the reaction wheel, $\theta$ is the angular velocity of the motor, $K_{\tau}$ is the torque constant of the motor, and $R_{i}$ is the interal resistance of the motor. The inertia tensor with respect to the center of the wheel and will be a function of the mass of the wheel but not the motor because the wheel in spinning in this case and the motor is considered stationary in the c-frame.

To get $\vec{\alpha}$, take the derivative of the angle of an arbitrary point on the wheel, Q. Usually, if the axis of rotation didn't match an direction vector in the inertial frame, taking the derivative would require kinematic decomposition (basic kinematic equation, BKE) where

$${}^{i}\frac{d}{dt}  {}^{b}(\cdot) =  {}^{b} \frac{d}{dt}(\cdot) + {}^{i}\omega^{b} \times  {}^{b}(\cdot)$$

where an i superscript denotes a derivative or object with respect to (wrt) the inertial frame, a b superscript denotes a derivative or object wrt the body frame, and $^{i}\omega^{b}$ is the angular velocity of the body frame in the inertial frame. The equation for the torque of the motor is derived from the velocity constant, $K_v$ that relates the velocity to the voltage for a brushed DC motor in the form

$$RPM = K_v * V$$ 

converting this to rad/s

$$ \omega = 2 \pi K_v / 60 $$

and converting to the torque constant

$$ K_{\tau} = \frac{1}{K_{v,rad/s}} $$

Usually taking derivatives in rotating frames requires the use of the BKE but is not required in our case because the angular velocities are lined up with an axis of the inertial frame which makes the cross product term = 0. Taking this derivative gives us

$$\vec{\alpha}_{w} = \ddot{\theta} \hat{c}_{3} = \ddot{\theta} \hat{k}$$

which gives us

$$\vec{\tau}_{w} = I_{w}\ddot{\theta} \hat{k} = \frac{K_{tau}}{R_{i}} V - \frac{K_{tau}^2}{R_{i}} \dot{\theta} $$

### Pendulum System

The torque generated by the reaction wheel can be seen as an applied torque in the opposite direction (Newton's law of action-reaction) when the system consists of the pendulum. Looking at the figure, it can be seen that the forces, or moments (torques) in our case, are the moment due to gravity as well as the torque supplied by the reaction wheel. Using the torque relationship presented before

$$\Sigma \vec{\tau} = \vec{\tau}_w + \vec{\tau}_g = I_w\ddot{\theta} (-\hat{k}) + \vec{\tau}_g$$
$$ = I_{p}\ddot{\phi}$$

Here, $\ddot{\phi}$ is derived in the same manner as $\ddot{\theta}$. The next problem will be determining to torque due to gravity on the pendulum.

It is known that torque due to gravity is equivalent to a moment arm with a force acting on the center of mass. Where the torque equation is given as

$$\vec{\tau} = \vec{r}_{CM} \times \vec{F}$$

which turns into 

$$\vec{\tau}_{g} = \ell \hat{b}_1 \times m_p g (-\hat{j})$$

Looks like we did use one of the position vectors from above. We will also use one of the rotation matrices (or a portion of it) determined above where

$$\hat{j} = cos(\phi) \hat{b}_1 - sin(\phi) \hat{b}_2$$

which changes our gravity torque

$$\vec{\tau}_{g} = \ell \hat{b}_1 \times m_p g (-(cos(\phi) \hat{b}_1 - sin(\phi) \hat{b}_2))$$

$$ = \ell m_p g sin(\phi) \hat{b}_3 = \ell m_p g sin(\phi) \hat{k}$$

Where $m_p$ is the mass of the pendulum arm, the motor, and the reaction wheel. Substituting this back into our pendulum torque gives us 

$$I_w\ddot{\theta} (-\hat{k}) + \vec{\tau}_g = -(\frac{K_{tau}}{R_{i}} V - \frac{K_{tau}^2}{R_{i}} \dot{\theta}) \hat{k} + \ell m_p g sin(\phi) \hat{k}$$

$$ = I_{p}\ddot{\phi}$$

This gives us the two equations of motion for the system with (including the equation of motion for the reaction wheel itself). Here we can determine if we want to include $\ddot{\theta}$ as a state variable or as an input. In this case, we will only care about the angle of the pendulum and its derivative. This gives us a state vector that has two elements: $\phi$ and $\dot{\phi}$. We will use a sensor on the motor to determine the angular velocity of the reaction wheel, but this will be done separately in order to validate our motor model (torque vs electrical input).

One last consideration is the friction about the rotation point, this can be added on as follows

$$ I_{p}\ddot{\phi} = -(\frac{K_{tau}}{R_{i}} V - \frac{K_{tau}^2}{R_{i}} \dot{\theta})\hat{k} + \ell m_p g sin(\phi) \hat{k} + \mu L \dot{\phi} \hat{k}$$

Where $\mu$ is the coefficient of friction, since we are using a smooth bearing, this value will be very small. This uses the method shown in this video [Friction Pendulum Video](https://www.youtube.com/watch?v=SZWn7x4g-Vo)

### State-Space

We will use various state-space methods to generate a control scheme for the reaction wheel, so conversion into state-space is a logical next step. To do so, we will take the state vector as

$$\vec{x} = \left\lbrack \begin{array}{c}
x_{1} \\
x_{2}
\end{array}\right\rbrack$$

$$ = \left\lbrack \begin{array}{c}
\phi \\
\dot{\phi} \\ 
\dot{\theta}
\end{array}\right\rbrack $$

then taking the derivative

$$\dot{\vec{x}} = \left\lbrack \begin{array}{c}
\dot{\phi} \\
\ddot{\phi}\\
\ddot{\theta}
\end{array}\right\rbrack = \left\lbrack \begin{array}{c}
\dot{x}_1 \\
\dot{x}_2 \\ 
\dot{x}_3
\end{array}\right\rbrack$$ 

$$ = \left\lbrack \begin{array}{c}
\dot{x_{2}} \\
I_{p}^{-1} (m_m g \ell sin(x_{1}) - \mu \ell x_{2} - \frac{K_{tau}}{R_{i}} V - \frac{K_{tau}^2}{R_{i}} \dot{\theta}\\
(K_t * u / R_i - K_t^2 * \dot{\theta} / R_i) / I_w 
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

Since all rotations will be along the third axis, $\hat{k} = \hat{b}_1 = \hat{c}_1$, only the bottom right element (3,3) will be used from each of the inertia tensors.

## Control
### Challenges
One of the challenges of controlling this system is the saturation of the motor. Since motors have a maximum RPM, the motors can't accelerate higher than this limit to apply additional torque. Traditional control methods such as pole placement and LQR are unable to set constraints on the maximum RPM of the motors into account causing potential control schemes that forces the input to go above the rated voltage or speed of the motor. One way to avoid this is to set a maximum voltage or speed for the motor in the arduino code and not allow it to go above, however, this doesn't negate the fact that the control law is still calculating such high values. A numerical method called model predictive control (in linear and nonlinear flavors) can be used alongside constraints in order to create a control scheme for the reaction wheel. The only problem is the intensive computation required to apply MPCs online.

### Linearization
To make use of linear control methods like pole placement and LQR, the system has to be linearized with respect to a point, whether it be a critical point such as an equilibrim point or the next step in a desired trajectory. To do so, a Taylor series expension will be utilized for the matrix equation. For a general state-space system

$$\vec{\dot{x}} = \vec{f}(\vec{x},\vec{u})$$

Taylor series expansion of the first order gives

$$\vec{f}(\vec{x},\vec{u}) \approx \vec{f}(\vec{x}_0,\vec{u}_0) + \frac{\partial{\vec{f}}}{\partial{\vec{x}}}(\vec{x}_0,\vec{u}_0) (\vec{x} - \vec{x}_0) + \frac{\partial{\vec{f}}}{\partial{\vec{u}}} (\vec{x}_0,\vec{u}_0) (\vec{u} - \vec{u}_0)$$

Where the first term is zero because it is evaluated at a stationary point (may be different for a tracking case). A is the first Jacobian multiplied by the x variation, $\delta \vec{x} = \vec{x} - \vec{x}_0$, and B is the second Jacobian multiplied by the input, u, variation, $\delta \vec{u} = \vec{u} - \vec{u}_0$ (here we will just take $\vec{u}_0 = 0$).

$$A=\left\lbrack \begin{array}{ccc}
\frac{\partial }{\partial x_1 }\vec{f}  & \ldotp \ldotp \ldotp  & \frac{\partial }{\partial x_n }\vec{f} 
\end{array}\right\rbrack =\left\lbrack \begin{array}{c}
\nabla^T f_1 \\
\vdots \\
\nabla^T f_n 
\end{array}\right\rbrack =\left\lbrack \begin{array}{ccc}
\frac{\partial f_1 }{\partial x_1 } & \cdots  & \frac{\partial f_1 }{\partial x_n }\\
\vdots  & \ddots  & \vdots \\
\frac{\partial f_n }{\partial x_1 } & \cdots  & \frac{\partial f_n }{\partial x_n }
\end{array}\right\rbrack$$

Similarly, taking the jacobian with respect to the inputs gives us the B matrix

$$B=\left\lbrack \begin{array}{ccc}
\frac{\partial f_1 }{\partial u_1 } & \cdots  & \frac{\partial f_1 }{\partial u_m }\\
\vdots  & \ddots  & \vdots \\
\frac{\partial f_n }{\partial u_1 } & \cdots  & \frac{\partial f_n }{\partial u_m }
\end{array}\right\rbrack$$

Where m is the number of inputs that a system has. For both matries, the partial derivatives will be evaluated at the determined ancor point. 

Doing these partial derivatives gives the following A and B matrices

$$A =  \left(\begin{array}{ccc}
0 & 1 & 0\\
\frac{g\,l\,\cos \left(x_1 \right)\,{\left(m_L +m_m +m_w \right)}}{I_p } & 0 & \frac{{K_t }^2 }{I_p \,R_i }\\
0 & 0 & -\frac{{K_t }^2 }{I_w \,R_i }
\end{array}\right)$$

$$B = \left(\begin{array}{c}
0\\
-\frac{K_t }{I_p \,R_i }\\
\frac{K_t }{I_w \,R_i }
\end{array}\right)$$

Here, the matrices will be evaluated at the equilibrium point which can be inferred to be either when the pendulum is pointed straight up or straight down (0 degrees or 180 degrees) and all rates are equal to zero, $\dot{\phi} = \dot{\theta} = 0$.

To even begin to think about creating linear control schemes, we have to check if the linear system is even controllable and observable.

### Pole Placement
One of the simplest methods of controlling a state-space system is to apply a pole placement. This control method takes

$$\vec{u}(t) = -K \vec{x}(t)$$ 

Where K is a row vector with the same length as B is tall and will be the gains for the system (the number of states in the system). Assuming the system state-space dynamics have been linearized in the form

$$\dot{\vec{x}}(t) = A \vec{x}(t) + B \vec{u}(t)$$

The input can be substituted in resulting in

$$\dot{\vec{x}}(t) = A \vec{x}(t) + B (-K \vec{x}(t)) = (A - BK)\vec{x}(t)$$

To make the system stable, the eigenvalues of the new state-space system have to be on the open left hand plane of the complex plane. This means that the real parts of the eigenvalues have to be strictly negative in order for the system to "decay" towards a critical point (the equilibrium point found in linearizing the system). To do so, determine the characteristic equation of the new system matrix $(A - BK)$ where the values of K are underdetermined 

$$ \chi_{A}(\lambda) = |\lambda I - A + BK| $$

Pole placement is the act choosing the poles or eigenvalues desired for a system. Choosing stable poles causes a system to be stable. Once the stable poles are chosen (2 needed for our system), create a desired characterisitc equation and set it equal to the system characteristic equation

$$ \chi_{d}(\lambda) = (\lambda - \lambda_1)(\lambda - \lambda_2)...(\lambda - \lambda_n) = |\lambda I - A + BK| $$

Expand the set of binomials of the left hand side and compute the determinant of the right hand side then solve for the undetermined elements in the K matrix. The numerical results can be seen below in the MATLAB results section.


### LQR

Like in the case for pole placement, LQR also utilizes inputs of the form $\vec{u}(t) = -K\vec{x}(t)$. The difference lies in the method of calculation of the K matrix. Solving for the gains involves the use of optimization where the cost function is (for the infinite horizon case)

$$J(x_0,u) = \int_{t_0 }^{\infty} x^T Q x + u^T R u\;dt$$

Here, we want to minimie the cost function while also subjecting it to the linearized dynamics of the system, $\dot{\vec{x}} = A\vec{x} + B\vec{u}$. To do so, it is important to note that the form of the cost function is an algebraic Riccati equation

$$ J = $$

### MPC

## Estimation
### Measurements
The specific sensors that will be used to measure the state of the system will be mentioned in a later section. An IMU will be used to measure the angular acceleration of the pendulum, a magnetic encoder will be assembled similarly to this [video](https://www.youtube.com/watch?v=hHTPMeXZcy0) in order to measure the position of the pendulum. Another magnetic encoder will also measure the RPM of the motor but will only be used to validate our motor model as mentioned before. This gives us 

$$\vec{y}(t) = C \vec{x}(t) = \left(\begin{array}{c}
1 & 0 & 0
\end{array}\right) \vec{x}(t)$$

Where C is a 1-by-3 matrix a one in the first entry as represent that we are only measuring the angle of the pendulum. These values will be determined by the calculation of processed measurement to usable data from the sensor.

### Extended Kalman Filter


## 3D Printing / CAD
Disclaimer: I am pretty bad at using CAD and creating models so these models may be terrible.

## Components
The section on the modeling took about a day to derive, formulate, and implement in MATLAB since I focused on dynamics and control in my undergrad. Bear with me with all this Arduino and electronics stuff, I'm trying to learn this on the fly to be able to implement all of this stuff.

### Arduino
### Motors
The motors that will be used are 12V DC brushed motors. They were cheap and came in a 2-pack and had a max unloaded RPM of 35000 according to the specifications. This gives us a $K_v% of 35000/12 which will be used later in the calculation of the torque that is able to be generated with the reaction wheel attatched. A 3s Li-Po (11.1V) battery will be used to power them so they will reach close enough to their maximum output.

### Motor Driver
Driving these motors will be a HiLetgo BTS7960. This allows the motors to be controlled by an Arduino and be able to switch between forward and reverse rotations. It takes in power from the battery and uses a pulse width modulation (PWM) signal from the Arduino to control the voltage going to the motors. Keep in mind that reversing a motor while its spinning could damage it through mechanical stresses as well causing electrical load spikes, fortunately for me, I don't intend to keep this project for long so it doesn't concern me too much.

### Sensors
#### MPU6050
The MPU 6050 is an integrated circut (IC) that contains a gyroscope and accelerometer. This means it can measure the angular rates and acceleration that its subject to. 
##### MPU6050 Functions (Arduino)

#### AS5600
The AS5600 is a magnetic encoder that is able to measure the orientation of a rotating magnet with placed close to the IC.

##### AS5600 Functions (Arduino)

## Build
### Wiring
### Assembly

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
