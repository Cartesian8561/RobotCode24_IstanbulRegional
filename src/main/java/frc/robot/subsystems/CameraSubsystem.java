// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  /** Creates a new CameraSubsystem. */
  boolean a = true;
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry  tv;
  NetworkTableEntry ty;
  NetworkTableEntry targetPoseToRobot;
  public CameraSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    tv = table.getEntry("tv");
    ty = table.getEntry("ty");
    targetPoseToRobot = table.getEntry("targetpose_robotpose");
  }

  public double getXOffset(){
    return tx.getDouble(0.0);
  }
  public double[] getTargetPose(){
    return targetPoseToRobot.getDoubleArray(new double[6]);
  }
  
  public double getYOffset(){
    return ty.getDouble(0.0);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("tx", getXOffset());
    SmartDashboard.putNumber("ty", getYOffset());
    // This method will be called once per scheduler run
  }

  public boolean isValid(){
    return tv.getBoolean(false);
  }


}