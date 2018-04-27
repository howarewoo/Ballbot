# Ballbot: a Dynamically Stable Omnidirectional Robot

*Under Construction*

TO DO:
- Manage direction pinout
- Refine PID control system
- Build and assemble chassis

A ballbot is a dynamically-stable mobile robot designed to balance on a single spherical wheel (i.e., a ball). Through its single contact point with the ground, a ballbot is omnidirectional and thus exceptionally agile, maneuverable and organic in motion compared to other ground vehicles. Its dynamic stability enables improved navigability in narrow, crowded and dynamic environments.

The ballbot works on the same principle as that of an inverted pendulum. As such, the ballbot can be broken down into three major components: The pendulum, the control system, and the drive unit. The pendulum component, and therefore the control system, can further be broken down from a single three dimensional coordinate system to pair of two dimensional coordinate systems.

Because the pendulum is broken down into two dimensional models, the control systems computes the required X and Y axis components to control the system. Using a linear quadratic regulator and discrete state space controller, the control system can use the angle from vertical to produce optimized control for the desired states. These states can be summed to represent a resultant vector in the XY plane.

This resultant vector can be converted to motor speeds though kinematic equations for the interaction between the motors, wheels, and ball.

CAPSTONE project by Adam Woo, Graham Goodwin, and Chloe Desjardins

## Mathematical Modeling

In essence, a ballbot is an omnidirectional inverted pendulum. This means that the ballbot can be broken down into two inverted pendulums that travel along the X and Y axis. The angle of the robot along these axis at any given time can be used to calculate the necessary velocity required to keep the pendulums upright.

The X and Y components can be summed to a desired resulting vector, which can be used to maintain the robots verticality in 3D space.

The state equations for an inverted pendulum can be represented as:

<img src="https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20%5Cdot%7Bx%7D%20%5C%5C%20%5Cddot%7Bx%7D%20%5C%5C%20%5Cdot%7B%5Ctheta%7D%20%5C%5C%20%5Cddot%7B%5Ctheta%20%7D%20%5Cend%7Bbmatrix%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%200%20%26%201%20%26%200%20%26%200%5C%5C%200%20%26%20-d/M%20%26%20-m*g/M%20%26%200%5C%5C%200%20%26%200%20%26%200%20%26%201%5C%5C%200%20%26%20-d/%28M*l%29%20%26%20-%28m&plus;M%29*g/%28M*l%29%20%26%200%20%5Cend%7Bbmatrix%7D%20%5Cbegin%7Bbmatrix%7D%20x%20%5C%5C%20%5Cdot%7Bx%7D%20%5C%5C%20%5Ctheta%20%5C%5C%20%5Cdot%7B%5Ctheta%20%7D%20%5Cend%7Bbmatrix%7D%20&plus;%20%5Cbegin%7Bbmatrix%7D%200%20%5C%5C%201/M%3B%20%5C%5C%200%20%5C%5C%201/%28M*l%29%20%5Cend%7Bbmatrix%7D%20u" class="center" />

<img src="https://latex.codecogs.com/gif.latex?y%20%3D%20%5Cbegin%7Bbmatrix%7D%200%20%26%201%20%26%200%20%26%200%20%5Cend%7Bbmatrix%7D%20%5Cbegin%7Bbmatrix%7D%20x%20%5C%5C%20%5Cdot%7Bx%7D%20%5C%5C%20%5Ctheta%20%5C%5C%20%5Cdot%7B%5Ctheta%20%7D%20%5Cend%7Bbmatrix%7D%20&plus;%20%5Cbegin%7Bbmatrix%7D%200%20%5Cend%7Bbmatrix%7D%20u" class="center"/>

Matlab was used for all of the control system modeling.


## Hardware

Here is a list of the selected hardware:

| Component | Part | Description |
| --- | --- | --- |
| Microcontroller | Teensy 3.6 | Faster processor speed, easy plug and play setup, and an extensive community make the Teensy 3.6 an ideal choice for the microcontroller over less powerful options like an Arduino Mega, or a less supported board. |
| IMU | MPU6050 | The 6 degrees of freedom supported by the MPU6050 allow for accurate angle measurements for a very cheap price. |
| Ball | 10" Stainless Steel Mirror Ball w/ Plastidip | A 10" steel ball provides a round ridged surface strong enough to support the robots chassis. The ball is coated in Plastidip, a rubberlike spray that will increase the friction on the ball. |
| Wheels | 3x 3.25" VexPro Omniwheel | Omniwheels allow for free movement perpendicular to the wheel. This allows for the development of holonomic movement systems. |
| Motors | 3x Nema 23 Stepper Motor | Stepper motors provide the high level of precision and torque at low speeds that the system will need to remain stable. |
| Motor Drivers | 3x TB6600 Stepper Motor Driver | These drivers allow the stepper motors to be controlled with only two data pins; one for direction and one for speed. |
| Battery | Gens ace 5000mAh 11.1V LiPo | The battery must be at least 9V. This battery was a highly rated battery with a large capacity, sufficient voltage, and relatively low weight. |
| Universal Battery Elimination Circuit | BW® RC Servo BEC UBEC 3A 5V | An UBEC is a voltage regulating circuit used to take power from the battery to the microprocessor. |

## Building Process

![TB6600 Wiring](https://github.com/awoox2/ballbot/raw/master/Images/TB6600_wiring.png)

The driver was manually set to the following:
- Microstepping: 1/32
- Current: 2.8A

The step pins output a square wave of varying frequencies. Pins 2, 3, and 5 where chosen because they all run on different timers. Therefore, the frequencies can be adjusted independently.

## Programming

All programming was done using the Atom text editor, Arduino IDE, and Teensyloader programs.

**Libraries**
- PID_v1.h by Brett Beauregard
- quaternionFilter.h by Kris Winer
- Math.h
- Wire.h
- FrequencyTimer2.h
- TimerOne.h
- TimerThree.h

## Results
