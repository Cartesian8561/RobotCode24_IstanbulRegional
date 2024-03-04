// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Helpers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous.AngleCalculator;
import frc.robot.commands.ShooterCommands.AutoShootCommand;
import frc.robot.commands.ShooterCommands.IntakeFeedCommand;
import frc.robot.commands.ShooterCommands.IntakeHoldCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/** Add your docs here. */
public class AutoHelpers {

    private SwerveSubsystem swerveSubsystem;
    private CameraSubsystem cameraSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intake;
    private ArmSubsystem armSubsystem;
    private AngleCalculator angleCalculator;
    private ColorSensor sensor;




    public AutoHelpers(SwerveSubsystem swerveSubsystem, CameraSubsystem cameraSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intake, ArmSubsystem armSubsystem, AngleCalculator angleCalculator, ColorSensor sensor){
        this.swerveSubsystem = swerveSubsystem;
        this.cameraSubsystem = cameraSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intake = intake;
        this.armSubsystem = armSubsystem;
        this.angleCalculator = angleCalculator;
        this.sensor = sensor;
    }


    public Command shootAuto(){
        ParallelRaceGroup command = new ParallelRaceGroup(
        new AutoShootCommand(shooterSubsystem, angleCalculator).withTimeout(1.5),
        new IntakeHoldCommand(armSubsystem, angleCalculator, sensor),
        new SequentialCommandGroup(
          new WaitCommand(0.6),
          new IntakeFeedCommand(intake)
        )
      );

      return command;
    }

    public RunCommand openArm(){
      return new RunCommand(() -> armSubsystem.moveArmPid(0.8), armSubsystem);

    }

    public ParallelRaceGroup closeArm(){
      return new RunCommand(() -> armSubsystem.moveArmPid(0.2), armSubsystem).withTimeout(1.2);
    }
}
