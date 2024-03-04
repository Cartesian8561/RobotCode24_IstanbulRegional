// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.TeleopAutos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class goToSpeaker extends Command {
  SwerveSubsystem swerveSubsystem;
  PIDController pidController = new PIDController(0.05, 0, 0);
  boolean isFinished;
  /** Creates a new goToAmp.*/
  public goToSpeaker(SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double steeringAdjust = MathUtil.clamp(pidController.calculate(swerveSubsystem.getEstimatedPose().getRotation().getDegrees(), swerveSubsystem.getSpeakerAngle()), -1.0, 1.0);
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, steeringAdjust * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond / 4, swerveSubsystem.getEstimatedPose().getRotation());
    SwerveModuleState[] swerveModuleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(swerveModuleState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    swerveSubsystem.stopModules();
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(swerveSubsystem.getEstimatedPose().getRotation().getDegrees() - swerveSubsystem.getSpeakerAngle()) < 5.0);
  }
}
