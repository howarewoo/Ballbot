# Ballbot: a Dynamically Stable Omnidirectional Robot

A ballbot is a dynamically-stable mobile robot designed to balance on a single spherical wheel (i.e., a ball). Through its single contact point with the ground, a ballbot is omnidirectional and thus exceptionally agile, maneuverable and organic in motion compared to other ground vehicles. Its dynamic stability enables improved navigability in narrow, crowded and dynamic environments. The ballbot works on the same principle as that of an inverted pendulum.

\- Wikipedia

CAPSTONE project by Adam Woo, Graham Goodwin, and Chloe Desjardins

## Mathematical Modeling

In essence, a ballbot is an omnidirectional inverted pendulum. This means that the problem can be approached in one of two ways.

1. Implement a 2-dimensional inverted pendulum control for each plane of force, and sum the results.
2. Find the angle for pitch around the z-axis, solve a 2-dimensional inverted pendulum problem for the plan along that angle, and then calculate the force needed for each point of control.


## Hardware

Here is a list of the selected hardware:

| Component | Part | Description |
| --- | --- | --- |
| Microcontroller | Raspberry Pi 3B | Faster processor speed and an extensive community make the Raspberry Pi an ideal choice for the microcontroller over less powerfull options like an Arduino Mega, or a less supported board. |
| IMU | MPU9250 | The 9 degrees of freedom supported by the MPU9250 allow for accurate angle measurments for a very cheap price. |

## Building Process

## Programming

## Results
