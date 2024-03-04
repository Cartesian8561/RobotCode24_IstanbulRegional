// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Autonomous.TeleopAutos.goToSpeaker;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {
  /** Creates a new SwerveJoystickCmd. */
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Double> fieldOrientedFunction;
  private final Supplier<Boolean> noteButton;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  PIDController pidController = new PIDController(0.05, 0, 0);
  PIDController noteController = new PIDController(0.003, 0, 0);

  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
      Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
      Supplier<Double> fieldOrientedFunction, Supplier<Boolean> noteButton) {
      this.swerveSubsystem = swerveSubsystem;
      this.xSpdFunction = xSpdFunction;
      this.ySpdFunction = ySpdFunction;
      this.turningSpdFunction = turningSpdFunction;
      this.fieldOrientedFunction = fieldOrientedFunction;
      this.noteButton = noteButton;
      this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
      addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
     double xSpeed = xSpdFunction.get();
      double ySpeed = ySpdFunction.get();
      double turningSpeed = turningSpdFunction.get();

      xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
      ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
      turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

      xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
      ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
      turningSpeed = turningLimiter.calculate(turningSpeed)
              * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    
    
    
    
    if((this.fieldOrientedFunction.get() < 0.5) && this.noteButton.get()){
        double offset = (swerveSubsystem.getNoteOffset() != 0) ? swerveSubsystem.getNoteOffset() : 240.0;
        SmartDashboard.putNumber("offset", offset);
        double steeringAdjust = MathUtil.clamp(noteController.calculate(offset, 240.0), -1.0, 1.0);
        SmartDashboard.putNumber("adjust", steeringAdjust);
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(-xSpeed, -ySpeed, steeringAdjust * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond / 3.0);
        SwerveModuleState[] swerveModuleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(swerveModuleState);
    }
    

    else if ((this.fieldOrientedFunction.get() < 0.5)) {

      ChassisSpeeds chassisSpeeds;

    
    if (true) {
          // Relative to field
          
          chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                  xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d()); 
            //chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);        
      } else {
          // Relative to robot
          chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
      }

      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

      swerveSubsystem.setModuleStates(moduleStates);
    }
    
    else {
        double steeringAdjust = MathUtil.clamp(pidController.calculate(swerveSubsystem.getXOffset(), 0.0), -1.0, 1.0);
        SmartDashboard.putNumber("allah offset", swerveSubsystem.getXOffset());
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, steeringAdjust * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond / 3.0, swerveSubsystem.getRotation2d());
        SwerveModuleState[] swerveModuleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(swerveModuleState);
    }

      
  }

  @Override
  public void end(boolean interrupted) {
      swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
      return false;
  }
}
