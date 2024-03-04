// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.Routines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous.AngleCalculator;
import frc.robot.Extras.no_use.Subsystems.ArmSubsystem;
import frc.robot.Helpers.AutoHelpers;
import frc.robot.Helpers.ColorSensor;
import frc.robot.Helpers.RobotPoses;
import frc.robot.commands.IntakeCommands.OpenIntakeCommand;
import frc.robot.commands.IntakeCommands.grabNote;
import frc.robot.commands.ShooterCommands.IntakeHoldCommand;
import frc.robot.commands.SwerveCommands.DriveToPoseCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpRoutine extends SequentialCommandGroup {
  AutoHelpers autoHelpers;
  SwerveSubsystem swerveSubsystem;
  CameraSubsystem cameraSubsystem;
  ShooterSubsystem shooterSubsystem;
  IntakeSubsystem intakeSubsystem;
  ArmSubsystem armSubsystem;
  AngleCalculator angleCalculator;
  ColorSensor colorSensor;
  RobotPoses robotPoses = new RobotPoses();


  /** Creates a new AmpRoutine. */
  public AmpRoutine(SwerveSubsystem swerveSubsystem, CameraSubsystem cameraSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intake, frc.robot.subsystems.ArmSubsystem armSubsystem, AngleCalculator angleCalculator, ColorSensor sensor) {
    this.autoHelpers = new AutoHelpers(swerveSubsystem, cameraSubsystem, shooterSubsystem, intake, armSubsystem, angleCalculator, sensor);
   
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeHoldCommand(armSubsystem, angleCalculator, sensor).withTimeout(1.7),
      autoHelpers.shootAuto(),
      new WaitCommand(0.3),

      new ParallelCommandGroup(
        new OpenIntakeCommand(armSubsystem, 21.6).withTimeout(1.5),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, robotPoses.yakinSagNotePose().getX(), robotPoses.yakinSagNotePose().getY(), robotPoses.yakinSagNotePose().getRotation().getDegrees()),
        new grabNote(intake,shooterSubsystem).withTimeout(2)
      ),
      new OpenIntakeCommand(armSubsystem, 0.0).withTimeout(1.5),
      new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, robotPoses.autoAmpPose().getX(), robotPoses.autoAmpPose().getY(), robotPoses.autoAmpPose().getRotation().getDegrees()),
      new ParallelRaceGroup(
        autoHelpers.openArm(),
        new SequentialCommandGroup(
          new WaitCommand(1.3),
          new RunCommand(() -> intake.setIntakeVoltage(-6.0), intake).withTimeout(1.2)
        ),
        new RunCommand(() -> intake.setIntakeVoltage(0.0), intake).withTimeout(1.2),
        autoHelpers.closeArm()
      )




    );
  }
}
