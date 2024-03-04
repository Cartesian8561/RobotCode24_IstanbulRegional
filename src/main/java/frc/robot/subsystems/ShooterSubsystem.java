// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Autonomous.AngleCalculator;
import frc.robot.Helpers.ColorSensor;

public class ShooterSubsystem extends SubsystemBase {

  CANSparkMax rightShooter;
  CANSparkBase leftShooter;
  ColorSensor sensor;
  
  
  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(ColorSensor colour) {
    rightShooter = new CANSparkMax(61, MotorType.kBrushless);
    rightShooter.setInverted(false);
  
    leftShooter = new CANSparkMax(62, MotorType.kBrushless);
    leftShooter.setInverted(true);
    this.sensor = colour;
  }

  public boolean matchColor(){
    return sensor.matchColor();
  }
  

  public Color getDetectedColor(){
      return sensor.getDetectedColour();
  }

  

  public void setShooterSpeed(double speed){
    rightShooter.set(speed);
    leftShooter.set(-speed);
  }

  public void setShooterVoltage(double volts){

    //TODO YARRAK
    rightShooter.setVoltage(volts);
    leftShooter.setVoltage(-volts);
    
  }

  public void stopMotors(){
    setShooterVoltage(0.0);
  }
  
  @Override
  public void periodic() {

    SmartDashboard.putNumber("r", getDetectedColor().red * 255.0f);
    //SmartDashboard.putNumber("g", getDetectedColor().green * 255.0f);
    //SmartDashboard.putNumber("b", getDetectedColor().blue * 255.0f);
    SmartDashboard.putBoolean("yes", matchColor());
  }
}
