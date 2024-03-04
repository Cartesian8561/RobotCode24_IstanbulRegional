// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class OpenIntakeCommand extends Command {
  /** Creates a new OpenIntakeCommand. */
  ArmSubsystem armSub;
  double setPoint;
  public OpenIntakeCommand(ArmSubsystem armSubsystem, double setPoint) {
    armSub = armSubsystem;
    addRequirements(armSubsystem);
    this.setPoint = setPoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSub.setLength(setPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.setOrtaVoltage(0.0);
    System.out.println("openitanke bitiş");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(armSub.getArmLength() - setPoint) < 0.8);
  }
}
