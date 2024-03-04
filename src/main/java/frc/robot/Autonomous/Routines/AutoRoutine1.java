// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.Routines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Autonomous.AngleCalculator;
import frc.robot.Helpers.AutoHelpers;
import frc.robot.Helpers.ColorSensor;
import frc.robot.Helpers.RobotPoses;
import frc.robot.commands.IntakeCommands.OpenIntakeCommand;
import frc.robot.commands.IntakeCommands.grabNote;
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
public class AutoRoutine1 extends SequentialCommandGroup {

  private RobotPoses robotPoses = new RobotPoses();
  private AutoHelpers autoHelpers;



  /** Creates a new AutoRoutine1. */
  public AutoRoutine1(SwerveSubsystem swerveSubsystem, CameraSubsystem cameraSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intake, ArmSubsystem armSubsystem, AngleCalculator angleCalculator, ColorSensor sensor) {
    this.autoHelpers = new AutoHelpers(swerveSubsystem, cameraSubsystem, shooterSubsystem, intake, armSubsystem, angleCalculator, sensor);
   
    addCommands(
      /*
      new SequentialCommandGroup(
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.1, 4.37, 27.44),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 13.8, 5.4, 0.0),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 13.87, 6.88, -27.55)
      )
    );
    */
  
    new SequentialCommandGroup(

    //eldekini at
      new IntakeHoldCommand(armSubsystem, angleCalculator, sensor).withTimeout(1.7),
      autoHelpers.shootAuto(),
      new WaitCommand(0.3),


      //new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.2, 4.3, 35.0),

      //yakin sag at
      new ParallelCommandGroup(
        new OpenIntakeCommand(armSubsystem, 21.6).withTimeout(1.5),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, robotPoses.yakinSagNotePose().getX(), robotPoses.yakinSagNotePose().getY(), robotPoses.yakinSagNotePose().getRotation().getDegrees()),
        new grabNote(intake,shooterSubsystem).withTimeout(2)
      ),
      autoHelpers.shootAuto(), 



      //yakin orta at
      new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, Constants.xAllianceFlip(14.4), 5.4, Constants.rotAllianceFlip(new Rotation2d(0)).getDegrees()), //atis icin yol duzenleme
      new ParallelCommandGroup(
        new OpenIntakeCommand(armSubsystem, 21.6).withTimeout(1.5),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, robotPoses.yakinOrtaNotePose().getX(), robotPoses.yakinOrtaNotePose().getY(), robotPoses.yakinOrtaNotePose().getRotation().getDegrees()),
        new grabNote(intake,shooterSubsystem).withTimeout(1.5)
      ),
      autoHelpers.shootAuto(),



      //yakin sol at
      new ParallelCommandGroup(
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, robotPoses.yakinSolNotePose().getX(), robotPoses.yakinSolNotePose().getY(), robotPoses.yakinSolNotePose().getRotation().getDegrees()),
        new grabNote(intake,shooterSubsystem).withTimeout(2.0),
        new OpenIntakeCommand(armSubsystem, 21.6).withTimeout(1.0)
      ),
      autoHelpers.shootAuto(),



      //uzak sag 1 at
      new ParallelCommandGroup(
      new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, robotPoses.uzakSag1notePose().getX(), robotPoses.uzakSag1notePose().getY(), robotPoses.uzakSag1notePose().getRotation().getDegrees()),
      new grabNote(intake, shooterSubsystem).withTimeout(4.0)
      ),
      new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, Constants.xAllianceFlip(12.02), 7.42f, Constants.rotAllianceFlip(new Rotation2d(Math.toRadians(-27.0))).getDegrees()),
      autoHelpers.shootAuto()
    )
    );








/*


      new ParallelCommandGroup(
        new SequentialCommandGroup(
        new OpenIntakeCommand(armSubsystem, 21.2),  
        new IntakeHoldCommand(armSubsystem, angleCalculator, sensor),
        new AutoShootCommand(shooterSubsystem, angleCalculator),
        new IntakeFeedCommand(intake),
        new ParallelCommandGroup(
          new IntakeHoldCommand(armSubsystem, angleCalculator, sensor),
          new IntakeFeedCommand(intake),
          new AutoShootCommand(shooterSubsystem, angleCalculator)
        ),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 13.88, 5.14, -27.44),
        new IntakeHoldCommand(armSubsystem, angleCalculator, sensor),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 13.4, 5.4, 0.0),
        new IntakeHoldCommand(armSubsystem, angleCalculator, sensor),
        new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 13.87, 6.88, -27.44),
        new IntakeHoldCommand(armSubsystem, angleCalculator, sensor)
        
        
        )*/
       
        //new SequentialCommandGroup(
          //,
          //new AutoShootCommand(shooterSubsystem, angleCalculator)
        //)
        
      

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands();
  }
}
