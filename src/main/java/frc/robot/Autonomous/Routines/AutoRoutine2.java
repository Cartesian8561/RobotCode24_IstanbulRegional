// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.Routines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous.AngleCalculator;
import frc.robot.Helpers.ColorSensor;
import frc.robot.commands.IntakeCommands.OpenIntakeCommand;
import frc.robot.commands.IntakeCommands.grabNote;
import frc.robot.commands.ShooterCommands.AutoShootCommand;
import frc.robot.commands.ShooterCommands.IntakeFeedCommand;
import frc.robot.commands.ShooterCommands.IntakeHoldCommand;
import frc.robot.commands.SwerveCommands.DriveToPoseCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRoutine2 extends SequentialCommandGroup {
  /** Creates a new AutoRoutine2. */
  public AutoRoutine2(SwerveSubsystem swerveSubsystem, CameraSubsystem cameraSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intake, ArmSubsystem armSubsystem, AngleCalculator angleCalculator, ColorSensor sensor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SequentialCommandGroup(
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 12.8, 7.0, 0),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 8.85, 7.1, 0),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 12.2, 5.9, 0),
        //new waitCommand(1),

        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 10.7, 6.4, 0),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 8.3, 5.3, 0),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 10.7, 6.4, 0),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 12.2, 5.9, 0).withTimeout(5),


        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 10.7, 6.9, 0),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 8.6, 3.7, 0),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 9.3, 6.3, 0),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 11.1, 7.2, 0)

      
      )
    );
  }
}
