// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class grabNote extends Command {

  IntakeSubsystem intake;
  long startTime = 0;
  boolean isFinished;
  ShooterSubsystem shooter;

  /** Creates a new TurnIntakeCommand. */
  public grabNote(IntakeSubsystem subsystem, ShooterSubsystem shooter) {
    this.shooter = shooter;
    this.intake = subsystem;
    addRequirements(subsystem);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = 0L;
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.matchColor() && startTime == 0L){
      startTime = System.currentTimeMillis();
    }

    if(startTime == 0L){
      intake.setIntakeVoltage(7.0);
    }
    else{
      if(System.currentTimeMillis() - startTime < 0.45){
        intake.setIntakeVoltage(-5);
        shooter.setShooterVoltage(-7);
      }else{
        intake.setIntakeVoltage(0.0);
        isFinished = true;
      }
    }

  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeVoltage(0.0);
    shooter.setShooterVoltage(0.0);
    System.out.println("grabnote biti");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
