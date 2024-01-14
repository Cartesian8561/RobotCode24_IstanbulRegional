// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private final SwerveModule frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDriveMotorPort,
    DriveConstants.kFrontLeftTurningMotorPort,
    DriveConstants.kFrontLeftDriveEncoderReversed,
    DriveConstants.kFrontLeftTurningEncoderReversed,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

private final SwerveModule frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveMotorPort,
    DriveConstants.kFrontRightTurningMotorPort,
    DriveConstants.kFrontRightDriveEncoderReversed,
    DriveConstants.kFrontRightTurningEncoderReversed,
    DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

private final SwerveModule backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveMotorPort,
    DriveConstants.kBackLeftTurningMotorPort,
    DriveConstants.kBackLeftDriveEncoderReversed,
    DriveConstants.kBackLeftTurningEncoderReversed,
    DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
    DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

private final SwerveModule backRight = new SwerveModule(
    DriveConstants.kBackRightDriveMotorPort,
    DriveConstants.kBackRightTurningMotorPort,
    DriveConstants.kBackRightDriveEncoderReversed,
    DriveConstants.kBackRightTurningEncoderReversed,
    DriveConstants.kBackRightDriveAbsoluteEncoderPort,
    DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

private final AHRS gyro = new AHRS(SPI.Port.kMXP);


private double rot = 0;

private CameraSubsystem cameraSubsystem;


private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
    new Rotation2d(0),
    new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    }
    );

private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, getRotation2d(), new SwerveModulePosition[]{
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition()
}, getPose());

public SwerveSubsystem(CameraSubsystem cameraSubsystem) {
new Thread(() -> {
    try {
        Thread.sleep(1000);
        zeroHeading();
    } catch (Exception e) {
    }
}).start();
this.cameraSubsystem = cameraSubsystem;
}

public void zeroHeading() {
    gyro.reset();
}


public double noAngle(){
    return 0.0;
}

public void resetOrientation(){
    zeroHeading();
}


public double getHeading() {
return Math.IEEEremainder(gyro.getAngle(), 360.0);
}

public Rotation2d getRotation2d() {
return Rotation2d.fromDegrees(getHeading());
}

public Pose2d getPose() {
    return odometer.getPoseMeters();
}

public Pose2d getEstimatedPose(){
    return m_poseEstimator.getEstimatedPosition();
}

public void resetPoseEstimator(Pose2d pose){
    m_poseEstimator.resetPosition(getRotation2d(), new SwerveModulePosition[]{
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    }, pose);
}

public void resetOdometry(Pose2d pose) {
  odometer.resetPosition(getRotation2d(),
    new SwerveModulePosition[] {
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition()
} , pose);
}

public Rotation2d desiredRot(){
    //return Rotation2d.fromDegrees(rot += 1.0);
    return new Rotation2d();
}

public void addVisionMeasurement(Pose2d visionMeasurement2d){
    m_poseEstimator.addVisionMeasurement(visionMeasurement2d, Timer.getFPGATimestamp());
}

@Override
public void periodic() {
odometer.update(getRotation2d(), 
new SwerveModulePosition[] {
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition()
});

m_poseEstimator.update(getRotation2d(), new SwerveModulePosition[]{
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition()
});

if(cameraSubsystem.get2dPose() != null && cameraSubsystem.isValid()){
    addVisionMeasurement(cameraSubsystem.get2dPose());
}
    //SmartDashboard.putNumber("estimated rot", getEstimatedPose().getRotation().getDegrees());

    SmartDashboard.putNumber("estimated x", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("estimated y", m_poseEstimator.getEstimatedPosition().getY());

    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    SmartDashboard.putNumber("front right magReading", frontRight.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("front left magReading", frontLeft.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("back right magReading", backRight.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("back left magReading", backLeft.getAbsoluteEncoderRad());

    SmartDashboard.putNumber("front right drive", frontRight.getDrivePosition());
    SmartDashboard.putNumber("front left drive", frontLeft.getDrivePosition());
    SmartDashboard.putNumber("back right drive", backRight.getDrivePosition());
    SmartDashboard.putNumber("back left drive", backLeft.getDrivePosition());

    
    SmartDashboard.putNumber("front right angle", frontRight.getTurningPosition());
    SmartDashboard.putNumber("front left angle", frontLeft.getTurningPosition());
    SmartDashboard.putNumber("back right angle", backRight.getTurningPosition());
    SmartDashboard.putNumber("back left angle", backLeft.getTurningPosition());

    SmartDashboard.putNumber("left back drive volts", backLeft.getDriveVolts());
    SmartDashboard.putNumber("left back turn volts", backLeft.getTurnVolts());

    SmartDashboard.putNumber("robot x", odometer.getPoseMeters().getX());
    SmartDashboard.putNumber("robot y", odometer.getPoseMeters().getY());
    SmartDashboard.putNumber("robot angle", odometer.getPoseMeters().getRotation().getRadians());
}

public void stopModules() {
frontLeft.stop();
frontRight.stop();
backLeft.stop();
backRight.stop();
}

public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
    }
}
