// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Helpers.LimelightHelpers;
import frc.robot.Helpers.FieldConstants.Speaker;
import frc.robot.Helpers.LimelightHelpers.LimelightResults;

public class CameraSubsystem extends SubsystemBase {
  /** Creates a new CameraSubsystem. */
  boolean a = true;
  NetworkTable table;
  NetworkTable vision;
  NetworkTableEntry orta;
  NetworkTableEntry confidence;
  NetworkTableEntry tx;
  NetworkTableEntry  tv;
  NetworkTableEntry ty;
  NetworkTableEntry tid;
  NetworkTableEntry targetPoseToRobot;
  NetworkTableEntry apriltagPose;
  
  public CameraSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    vision = NetworkTableInstance.getDefault().getTable("photonvision/A4tech_FHD_1080P_PC_Camera");
    orta = vision.getEntry("targetPixelsX");
    tx = table.getEntry("tx");
    tv = table.getEntry("tv");
    ty = table.getEntry("ty");
    tid = table.getEntry("tid");

    targetPoseToRobot = table.getEntry("botpose_wpiblue");
    apriltagPose = table.getEntry("botpose_targetspace");

  }



  public double getNote(){
    return orta.getDouble(0.0);
  }
  


  public int getNumOfTargets(){
    return LimelightHelpers.getLatestResults("limelight").targetingResults.targets_Fiducials.length;

  }

  public double getXOffset(){
    return tx.getDouble(0.0);
  }
  public double[] getTargetPose(){
    return targetPoseToRobot.getDoubleArray(new double[6]);
  }

  public Pose3d get3dPose(){
    double[] pose = getTargetPose();
    return new Pose3d(pose[0], pose[1], pose[2], new Rotation3d(pose[3], pose[4], pose[5]));
  }

  public Pose2d get2dPose(){
    return new Pose2d(get3dPose().getX(),get3dPose().getY(), new Rotation2d());
  }

  public double[] getApriltagTargetPose(){
    return apriltagPose.getDoubleArray(new double[6]);
  }

  public Pose3d getApriltag3dPose(){
    double[] pose = getApriltagTargetPose();
    return new Pose3d(pose[0], pose[1], pose[2], new Rotation3d(pose[3], pose[4], pose[5]));
  }

  public Pose2d getApriltag2dPose(){
    return new Pose2d(getApriltag3dPose().getZ(), getApriltag3dPose().getX(), new Rotation2d());
  }
  
  public double getYOffset(){
    return ty.getDouble(0.0);
  }

  public double getID(){
    return tid.getInteger(0);
  }
  
  @Override
  public void periodic() {
    //SmartDashboard.putNumber("tx", getXOffset());
    //SmartDashboard.putNumber("ty", getYOffset());
    //SmartDashboard.putNumber("tvANNE", isNumValid());
    //SmartDashboard.putNumber("ikis2", get2dPose().getRotation().getDegrees());
    //SmartDashboard.putNumber("allahsiz limelight", getNumOfTargets());
    SmartDashboard.putNumber("annenx", getNote());
    // This method will be called once per scheduler run
  }

  public boolean isValid(){
    return tv.getBoolean(false);
  }

  public long isNumValid(){

    return tv.getInteger(0);
  }


}