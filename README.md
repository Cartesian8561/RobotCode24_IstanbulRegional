# RobotCode24
Cartesian Robotics official Robot Code for 2024 season Crescendo. Crescendo's code is written in Java with WPILib's java library.

## Code Basics
- Pose Estimation
The 2024 robot uses pose estimation of limelight by integrating aprltags on top of the normal odometry. The robot has auto-align commands to align with an apriltag on the field using PIDs.
- Swerve Modules
The robot uses a swerve drivetrain and uses swerve modules to accurately drive using the swerve drivetrain.
- Autonomous
The robot uses the WPILib's default clamped cubic spline generator and follows trajectories using a swerve controller command.
