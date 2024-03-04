// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Helpers.ColorSensor;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private WPI_TalonFX intake = new WPI_TalonFX(6);
  private ColorSensor sensor;
  public IntakeSubsystem(ColorSensor colour) {
    intake.setInverted(true);
    this.sensor = colour;
  }

  public void setIntakeVoltage(double volts){
    intake.setVoltage(volts);
  }

 public void stopMotor() {
  setIntakeVoltage(0.0);
 }
 
 public boolean matchColor() {
  return sensor.matchColor();
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
