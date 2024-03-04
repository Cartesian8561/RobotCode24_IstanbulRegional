// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;


public class DriveToPoseCommand extends Command {

  PIDController  pidController = new PIDController(0.05, 0, 0);
  CameraSubsystem cameraSubsystem;
  SwerveSubsystem swerveSubsystem;
  PIDController xmovingController = new PIDController(1.3, 0, 0);//4.8
  PIDController ymovingController = new PIDController(1.3, 0, 0);//1.5
  double xSetpoint;
  double ySetpoint;
  double angle;
  
  /** Creates a new AimAtTargetCommand. */
  public DriveToPoseCommand(CameraSubsystem camsubsystem ,  SwerveSubsystem swerveSubsystem, double xSetpoint, double ySetpoint, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    pidController.enableContinuousInput(-180, 180);
    this.cameraSubsystem = camsubsystem;
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(cameraSubsystem , swerveSubsystem);
    this.xSetpoint = xSetpoint;
    this.ySetpoint = ySetpoint;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      //double movingx = MathUtil.clamp(xmovingController.calculate(cameraSubsystem.getYOffset(), 15.0) / 20.0, -1.0, 1.0);
      //double movingy = MathUtil.clamp(ymovingController.calculate(-cameraSubsystem.getXOffset()) / 20.0, -1.0, 1.0);
      double movingx = MathUtil.clamp(xmovingController.calculate((swerveSubsystem.getEstimatedPose().getX()), xSetpoint), -1, 1.0);
      double movingy = MathUtil.clamp(ymovingController.calculate((swerveSubsystem.getEstimatedPose().getY()), ySetpoint), -1, 1.0);

      double steeringAdjust = MathUtil.clamp(pidController.calculate(swerveSubsystem.getEstimatedPose().getRotation().getDegrees(), angle), -1.0, 1.0);
      //double steeringAdjust = -MathUtil.clamp(pidController.calculate((swerveSubsystem.getEstimatedPose().getRotation().getDegrees()), 0) / 20.0, -1.0, 1.0);
      //SmartDashboard.putNumber("ye", movingy);
      //SmartDashboard.putNumber("iks", movingx);
      //SmartDashboard.putNumber("stter", steeringAdjust);
      //SmartDashboard.putNumber("estimated GET", swerveSubsystem.getEstimatedPose().getX());
      //SmartDashboard.putNumber("steer", steeringAdjust);
      //ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(movingx * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / 2, 0.0,steeringAdjust * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond / 4, swerveSubsystem.getRotation2d());
      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(movingx * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / 1.5, movingy * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / 1.5, steeringAdjust * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond / 4, swerveSubsystem.getEstimatedPose().getRotation());
    // steering value :: steeringAdjust * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond / 4
      SwerveModuleState[] swerveModuleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      swerveSubsystem.setModuleStates(swerveModuleState);

  } //önemli

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
    System.out.println("drivetopose bitş");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finised = (Math.abs(xSetpoint - swerveSubsystem.getEstimatedPose().getX()) + Math.abs(ySetpoint - swerveSubsystem.getEstimatedPose().getY()) < 0.25 && Math.abs(swerveSubsystem.getEstimatedPose().getRotation().getDegrees() - angle) < 5.0);

    return finised;
  }
}
