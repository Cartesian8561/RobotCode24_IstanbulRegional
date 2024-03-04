// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Autonomous.AngleCalculator;
import frc.robot.Constants.DahaIyiField;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCommand extends Command {
  /** Creates a new AutoShootCommand. */
  ShooterSubsystem shooter;
  boolean hasSeen;
  AngleCalculator calculator;

  public AutoShootCommand(ShooterSubsystem shooterSubsystem, AngleCalculator angleCalculator) {
    this.shooter = shooterSubsystem;
    calculator = angleCalculator;
    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasSeen = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.matchColor()){
      hasSeen = true;
    }
    double speed = 0;
    double distanceDelta = Math.abs(DahaIyiField.speaker.getX() - calculator.getRobotX());

    if (distanceDelta > 2.6 && distanceDelta < 4.4){
      speed = (0.8);
    }else if (distanceDelta >= 4.4) {
      speed = (0.85);
    }else{
      speed = (0.7);
    }

    //double speed = 0.6;
    SmartDashboard.putNumber("distance", calculator.calculateDistance(DahaIyiField.speaker));
    SmartDashboard.putNumber("shooter speed", speed);
    SmartDashboard.putNumber("asdklş", calculator.calculateIntakeAngle(DahaIyiField.speaker));
    shooter.setShooterSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooterVoltage(0.0);
    System.out.println("autoshoot bitiş");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (hasSeen && !shooter.matchColor());
  }
}










    /*if (DriverStation.getAlliance().get() == Alliance.Red){
      if (calculator.getRobotX() < 13.7) {
        speed = (0.8);
      }else if(calculator.getRobotX() < 11.9){
        speed = (0.85);
      } else{
        speed = (0.7);      
      }      
    }else{
      if (calculator.getRobotX() > Constants.xAllianceFlip(13.7)) {
        speed = (0.8);
      }else if(calculator.getRobotX() > Constants.xAllianceFlip(11.9)){
        speed = (0.85);
      } else{
        speed = (0.7);      
      }
    }*/