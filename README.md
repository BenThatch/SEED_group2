# SEED_group2

Group Members : Ben Thatcher, Gillian Yost, Chirs Hartsen, Dan Parr, Ruiqi Sun

Summary of each file in main repository branch

Demo01_StrightControl: Arduino sketch that is final Demo 1 code that takes a distance in feet and drives straight to that distance using a PID Controller and ramp response.

Demo01_TurnControl: Arduino sketch that is final Demo 1 code that takes an angle in degrees, moves to that angle and drives forward 1 foot using a PID Controller and ramp response.

Demo1_ISR_rewrit.ino : an arduino sketch adapted from the code Ben Wrote for assignment 2, which moniterd the movement of a virtual robot according to how 2 encoders were moved, to moniter how our actual wheels are moving. Main advantage over use of Encoder library velocity calculation in ISR.

Demo01_motor_driver.ino: An Arduino sketch using isr instead of Encoder.h with a PID controller to move the robot straight. This code does not work as intended, thus we returned to using Encoder.h with a ramp response.

PIDController.ino: Arduino sketch with PID controller from mini-project adjsuted to drive 2 motors for both wheels of the robot

PIDController_2Wheels.ino: Arduino sketch with PID controller updated from mini-project for 2 motors and a ramp response instead of step response.

Testcode_10_16.ino: combonation of the above listed arduino scripts to estimate position of the robot

angle_detection.py: Python code for raspberry pi to detect angle of aruco marker. The angle is positive it to left and negative to the right. Prints angle to LCD screen.

