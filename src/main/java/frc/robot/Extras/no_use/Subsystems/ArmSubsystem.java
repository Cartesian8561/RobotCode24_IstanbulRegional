// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Extras.no_use.Subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
  /** Creates a new ArmSubsystem. */
  CANSparkMax armMotor = new CANSparkMax(4, MotorType.kBrushless);
  RelativeEncoder encoder = armMotor.getEncoder();
  double feedforwardVolts = 0;
  ArmFeedforward feedforward = new ArmFeedforward(0.0, 1.96, 0.78);
  public PIDController pid = new PIDController(1.8, 0.0, 0.0);
  private static double kDt = 0.02;
  private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75,0.35));
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  
  public ArmSubsystem() {
    encoder.setPosition(0);
  }



  public State useState(){

    goal = new TrapezoidProfile.State(0.4, 0);

    setpoint = profile.calculate(kDt ,setpoint, goal);
    SmartDashboard.putNumber("setPoint velocity", setpoint.velocity);
    SmartDashboard.putNumber("setPoint Positoin", setpoint.position);



    return setpoint;
  }

  public void runTo(){
    double gain = feedforward.calculate(encoder.getPosition() ,useState().velocity) + pid.calculate(getArmAngle(), useState().position);
    setSpeed(gain);
    SmartDashboard.putNumber("ggain", gain);
  }

  public void setSpeed(double speed){
    armMotor.set(speed);
  }

  public void setVoltage(double volts){
    armMotor.setVoltage(volts);
  }

  public void holdArm(){
    armMotor.setVoltage(feedforward.calculate(encoder.getPosition(), 0));
  }

 

  public double getArmAngle(){
    return encoder.getPosition() * Math.PI * 2.0 / 20.0;
  }



  @Override
  public void periodic() {
  }


}
