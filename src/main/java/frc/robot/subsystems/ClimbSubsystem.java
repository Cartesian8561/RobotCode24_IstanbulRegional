// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private Servo climbServo = new Servo(0);
  private Servo climbServo2 = new Servo(1);
  private final CANSparkMax climberRight = new CANSparkMax(27, MotorType.kBrushless);
  private final CANSparkMax climberLeft = new CANSparkMax(28, MotorType.kBrushless);
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveServo(double setpoint){
    climbServo.setAngle(setpoint);
    climbServo2.setAngle(-setpoint);
  }

  public void setClimbVolts(double volts){
    climberRight.setVoltage(volts);
    climberLeft.setVoltage(-volts);
  }

  public void zeroMotors(){
    //climbServo.setSpeed(0);
    //climbServo2.setSpeed(0);
    climberRight.setVoltage(0);
    climberLeft.setVoltage(0);
  }
}
