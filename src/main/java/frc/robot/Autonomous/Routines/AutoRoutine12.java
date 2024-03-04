// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.Routines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous.AngleCalculator;
import frc.robot.Helpers.AutoHelpers;
import frc.robot.Helpers.ColorSensor;
import frc.robot.Helpers.RobotPoses;
import frc.robot.commands.IntakeCommands.OpenIntakeCommand;
import frc.robot.commands.IntakeCommands.grabNote;
import frc.robot.commands.SwerveCommands.DriveToPoseCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRoutine12 extends SequentialCommandGroup {


  private RobotPoses robotPoses = new RobotPoses();
  private AutoHelpers autoHelpers;

  /** Creates a new AutoRoutine12. */
  public AutoRoutine12(SwerveSubsystem swerveSubsystem, CameraSubsystem cameraSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intake, ArmSubsystem armSubsystem, AngleCalculator angleCalculator, ColorSensor sensor) {
    autoHelpers = new AutoHelpers(swerveSubsystem, cameraSubsystem, shooterSubsystem, intake, armSubsystem, angleCalculator, sensor);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //eldekini at
      new SequentialCommandGroup(
      new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, robotPoses.autoGarantiShootPose().getX(), robotPoses.autoGarantiShootPose().getY(), robotPoses.autoGarantiShootPose().getRotation().getDegrees()),
      new OpenIntakeCommand(armSubsystem, 21.6).withTimeout(1.7),
      autoHelpers.shootAuto(),


      //yakin sag at
      new ParallelCommandGroup(
        new OpenIntakeCommand(armSubsystem, 21.6).withTimeout(1.7),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, robotPoses.yakinSolNotePose().getX(), robotPoses.yakinSolNotePose().getY(), robotPoses.yakinSolNotePose().getRotation().getDegrees()),
        new grabNote(intake, shooterSubsystem).withTimeout(1.7).withTimeout(2)
      ),
      new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, robotPoses.autoGarantiShootPose().getX(), robotPoses.autoGarantiShootPose().getY(), robotPoses.autoGarantiShootPose().getRotation().getDegrees()),
      autoHelpers.shootAuto(),


      //yakin orta at
      new ParallelCommandGroup(
        new OpenIntakeCommand(armSubsystem, 21.6).withTimeout(1.7),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, robotPoses.yakinOrtaNotePose().getX(), robotPoses.yakinOrtaNotePose().getY(), robotPoses.yakinOrtaNotePose().getRotation().getDegrees()),
        new grabNote(intake, shooterSubsystem).withTimeout(2)
      ),
      new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, robotPoses.autoGarantiShootPose().getX(), robotPoses.autoGarantiShootPose().getY(), robotPoses.autoGarantiShootPose().getRotation().getDegrees()),
      autoHelpers.shootAuto(),



      //yakin sol at
      new ParallelCommandGroup(
        new OpenIntakeCommand(armSubsystem, 21.6).withTimeout(1.7),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, robotPoses.yakinSagNotePose().getX(), robotPoses.yakinOrtaNotePose().getY(), robotPoses.yakinSagNotePose().getRotation().getDegrees()),
        new grabNote(intake, shooterSubsystem).withTimeout(2)
      ),
      new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, robotPoses.autoGarantiShootPose().getX(), robotPoses.autoGarantiShootPose().getY(), robotPoses.autoGarantiShootPose().getRotation().getDegrees()),
      autoHelpers.shootAuto()
    )
  );
  }
}
