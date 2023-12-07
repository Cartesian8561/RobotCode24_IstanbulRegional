// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {

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
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 36;
        public static final int kBackLeftDriveMotorPort = 37;
        public static final int kFrontRightDriveMotorPort = 32;
        public static final int kBackRightDriveMotorPort = 35;

        public static final int kFrontLeftTurningMotorPort = 30;
        public static final int kBackLeftTurningMotorPort = 31;
        public static final int kFrontRightTurningMotorPort = 33;
        public static final int kBackRightTurningMotorPort = 34;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 3;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 4;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;
/* 
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 1.39;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 3.37;//3.37
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 1.31;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 5.61;
*/
 
/* 
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 1.38;//4.500;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.30;//3.431;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.84;//4.444;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 2.44;//2.528;

    */

        
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 1.26 - Math.PI;//1.40;//4.500;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -2.85 + Math.PI;//-2.74;//3.431;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 1.33 - Math.PI;//1.25;//4.444;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -0.64 + Math.PI;//2.53;//2.528;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.0;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 3.0;//4
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 2;//4
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 1; //hızını buradan arttırabiliyorsun
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2.5; // /10
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
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
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.09;
    }
}