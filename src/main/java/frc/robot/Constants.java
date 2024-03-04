// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public final class Constants {

    public static final class Offsets {
        public static final double xOffset = 0;
        public static final double yOffset = 0; 
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); //tik
        public static final double kDriveMotorGearRatio = 1 / 6.12; //tik
        public static final double kTurningMotorGearRatio = 1 / 12.8; //tik
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters; 
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.4;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = 0.43;
        // Distance between right and left wheels
        public static final double kWheelBase = 0.43;
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 37;
        public static final int kBackLeftDriveMotorPort = 32;
        public static final int kFrontRightDriveMotorPort = 35;
        public static final int kBackRightDriveMotorPort = 36;

        public static final int kFrontLeftTurningMotorPort = 31;
        public static final int kBackLeftTurningMotorPort = 33;
        public static final int kFrontRightTurningMotorPort = 34;
        public static final int kBackRightTurningMotorPort = 30;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        //double check rotation encoders

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 3;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 4;
        public static final int kBackRightDriveAbsoluteEncoderPort = 1;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        /* 
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 1.86;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 1.26;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -2.16;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -1.84;
        */


        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 1.85;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 1.20;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.96;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.29;
        

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.0;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2.0;//4
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                kPhysicalMaxAngularSpeedRadiansPerSecond / 2;//4
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final Transform2d robotToCam = new Transform2d(-0.30,-0.25, new Rotation2d());
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 16; //hızını buradan arttırabiliyorsun
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2.5; // /10
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;//4
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3; //3

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 4;
        public static final int kDriverNoteButtonIdx = 10;

        public static final double kDeadband = 0.09;
    }

    public static final class DahaIyiField {
        public static Rotation2d fieldFront = (DriverStation.getAlliance().get() == Alliance.Red) ? new Rotation2d() : new Rotation2d(Math.PI);
        public static Translation2d speaker = allianceFlip(new Translation2d(16.54, 5.66)); //15.61 5.55
    }
    
    public static double fieldLength = 16.54;

/* Orjinali red ise degistiriyordu calismama olasiligi var */
    public static Translation2d allianceFlip(Translation2d translation) {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
        return new Translation2d(fieldLength - translation.getX(), translation.getY());
        } else {
        return translation;
        }
    }

/* Orjinali red ise degistiriyordu calismama olasiligi var */
    public static Pose2d allianceFlip(Pose2d pose) {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
        return new Pose2d(
            fieldLength - pose.getX(),
            pose.getY(),
            new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
        } else {
        return pose;
        }
    }

    public static double xAllianceFlip(double x){
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            return (fieldLength - x);
        }else{
            return x;
        }
    }

    public static Rotation2d rotAllianceFlip(Rotation2d rot){
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            return (new Rotation2d(-rot.getCos(), rot.getSin()));
        }else{
            return rot;
        }
    }
}