// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
  /** Creates a new ArmSubsystem. */
  CANSparkMax armMotor = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax ortaMotor = new CANSparkMax(10, MotorType.kBrushed);
  RelativeEncoder encoder = armMotor.getEncoder();
  private Encoder armLengthCoder = new Encoder(0, 1);

  private PIDController ortaCommand = new PIDController(3.5, 0.0, 0.2);//2.1
  private final PIDController m_arm_pid = new PIDController(2.4, 0, 0.0); //kd yoktu
  private final double voltConstant = 1.3;
  public ArmSubsystem() {
    encoder.setPosition(0);
  }

  public void setLength(double setPoint){
    
    double cmd = -MathUtil.clamp(ortaCommand.calculate(getArmLength(), setPoint), -6.0, 6.0);
    if (Math.abs(getArmLength() - setPoint) < 0.02) cmd = 0;
    //double cmd = ortaCommand.calculate(getArmLength(), setPoint);
    SmartDashboard.putNumber("cmd", cmd);
    setOrtaVoltage(cmd);
  }

  
  public double getArmLength(){
    double a = armLengthCoder.getDistance()/1024;
    return a;
  }

  private double getHolderVolts(){
    double volts = voltConstant * Math.cos(getArmAngle() + Math.toRadians(30));
    
    return volts;
  }

  public void setOrtaVoltage(double volts){
    ortaMotor.setVoltage(volts);
  }

  public void moveArmPid(double setpoint){
    double volt = MathUtil.clamp(m_arm_pid.calculate(getArmAngle(), setpoint)+ getHolderVolts(), -10.0, 10.0);
    SmartDashboard.putNumber("annen", volt);
    setArmVoltage(volt);
  }


  public void setSpeed(double speed){
    armMotor.set(speed);
  }

  public void setArmVoltage(double volts){
    armMotor.setVoltage(volts);
  }

  public void resetArmLengthEncoder(){
    armLengthCoder.reset();
  }

 

  public double getArmAngle(){
    return encoder.getPosition() * Math.PI * 2.0 / (16.0 * 48.0 / 17.0);//* Math.PI * 2.0 / 20.0
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm encoder", getArmAngle());
    SmartDashboard.putNumber("arm length", getArmLength());
    // This method will be called once per scheduler run
  }


}