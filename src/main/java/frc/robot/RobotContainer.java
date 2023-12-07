// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AimAtTargetCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final CameraSubsystem cameraSubsystem = new CameraSubsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, 
    () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis), //buna bak
    () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
    () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
    () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
    configureBindings();
  }

  private void configureBindings() {

    new JoystickButton(driverJoystick, 1).onTrue(new InstantCommand(() -> swerveSubsystem.resetOrientation()));
    new JoystickButton(driverJoystick, 2).whileTrue(new AimAtTargetCommand(cameraSubsystem, swerveSubsystem)).onFalse(new InstantCommand(() -> swerveSubsystem.stopModules()));

  }

  public Command getAutonomousCommand() {

    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);
    
     /* 
    Trajectory trajectory = new Trajectory();
    String trajectoryJSON = new String("output/guzelsekiz.wpilib.json");
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
   }
    */
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(), new Pose2d(6.0,0.0, new Rotation2d(0)), config);
   //Trajectory trajectory =  TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new Rotation2d()), List.of(new Translation2d(0,1), new Translation2d(1,1), new Translation2d(1,0)), new Pose2d(0,0, new Rotation2d(0)), config);
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController,swerveSubsystem::desiredRot
    ,swerveSubsystem::setModuleStates, swerveSubsystem);


    swerveSubsystem.resetOdometry(trajectory.getInitialPose());
    
    return swerveControllerCommand.andThen(new InstantCommand(() -> swerveSubsystem.stopModules(), swerveSubsystem));
  }
}
