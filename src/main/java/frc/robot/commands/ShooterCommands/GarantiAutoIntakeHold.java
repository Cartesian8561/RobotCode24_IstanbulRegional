// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Autonomous.AngleCalculator;
import frc.robot.Constants.DahaIyiField;
import frc.robot.Helpers.ColorSensor;
import frc.robot.subsystems.ArmSubsystem;

public class GarantiAutoIntakeHold extends Command {
  /** Creates a new IntakeHoldCommand. */
  private ArmSubsystem armSubsystem;
  private AngleCalculator calculator;
  private ColorSensor sensor;

  public GarantiAutoIntakeHold(ArmSubsystem armSubsystem, AngleCalculator angleCalculator, ColorSensor sensor) {
    this.armSubsystem = armSubsystem;
    this.sensor = sensor;
    addRequirements(armSubsystem);
    calculator = angleCalculator;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setLength(calculator.calculateIntakeAngle(DahaIyiField.speaker));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setOrtaVoltage(0.0);
    System.out.println("intakeold bitiş");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !sensor.matchColor();
  }
}
