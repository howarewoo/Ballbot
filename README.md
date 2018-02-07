# Ballbot: a Dynamically Stable Omnidirectional Robot

*Under Construction*

A ballbot is a dynamically-stable mobile robot designed to balance on a single spherical wheel (i.e., a ball). Through its single contact point with the ground, a ballbot is omnidirectional and thus exceptionally agile, maneuverable and organic in motion compared to other ground vehicles. Its dynamic stability enables improved navigability in narrow, crowded and dynamic environments. The ballbot works on the same principle as that of an inverted pendulum.

\- Wikipedia

CAPSTONE project by Adam Woo, Graham Goodwin, and Chloe Desjardins

## Mathematical Modeling

In essence, a ballbot is an omnidirectional inverted pendulum. This means that the ballbot can be broken down into two inverted pendulums that travel along the X and Y axis. The angle of the robot along these axis at any given time can be used to calculate the necessary velocity required to keep the pendulums upright.

The X and Y components can be summed to a desired resulting vector, which can be used to maintain the robots verticality in 3D space.

Matlab was used for all of the control system modeling.

## Hardware

Here is a list of the selected hardware:

| Component | Part | Description |
| --- | --- | --- |
| Microcontroller | Teensy 3.6 | Faster processor speed, easy plug and play setup, and an extensive community make the Teensy 3.6 an ideal choice for the microcontroller over less powerful options like an Arduino Mega, or a less supported board. |
| IMU | MPU9250 | The 9 degrees of freedom supported by the MPU9250 allow for accurate angle measurements for a very cheap price. |
| Ball | 10" Stainless Steel Mirror Ball w/ Plastidip | A 10" steel ball provides a round ridged surface strong enough to support the robots chassis. The ball is coated in Plastidip, a rubberlike spray that will increase the friction on the ball. |
| Wheels | 3x 3.25" VexPro Omniwheel | Omniwheels allow for free movement perpendicular to the wheel. This allows for the development of holonomic movement systems. |
| Motors | 3x Nema 23 Stepper Motor | Stepper motors provide the high level of precision and torque at low speeds that the system will need to remain stable. |
| Motor Drivers | 3x TB6600 Stepper Motor Driver | These drivers allow the stepper motors to be controlled with only two data pins; one for direction and one for speed. |
| Battery | Gens ace 5000mAh 11.1V LiPo | The battery must be at least 9V. This battery was a highly rated battery with a large capacity, sufficient voltage, and relatively low weight. |
| Universal Battery Elimination Circuit | BW® RC Servo BEC UBEC 3A 5V | An UBEC is a voltage regulating circuit used to take power from the battery to the microprocessor. |

## Building Process

![TB6600 Wiring](https://github.com/awoox2/ballbot/raw/master/Images/TB6600_wiring.png)

## Programming

All programming was done in the Atom text editor with the PlatformIO IDE package.

**Libraries**
- AccelStepper by Mike McCauley
- PID by Brett Beauregard

## Results
