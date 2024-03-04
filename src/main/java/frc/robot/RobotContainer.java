// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autonomous.AngleCalculator;
import frc.robot.Autonomous.Routines.AmpRoutine;
import frc.robot.Autonomous.Routines.AutoRoutine1;
import frc.robot.Autonomous.Routines.AutoRoutine12;
import frc.robot.Autonomous.Routines.AutoRoutine2;
import frc.robot.Autonomous.Routines.besdakkatest;
import frc.robot.Autonomous.TeleopAutos.goToSpeaker;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DahaIyiField;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Helpers.ColorSensor;
import frc.robot.commands.IntakeCommands.CloseArmCommand;
import frc.robot.commands.IntakeCommands.grabNote;
import frc.robot.commands.ShooterCommands.AutoShootCommand;
import frc.robot.commands.SwerveCommands.AimAtTargetCommand;
import frc.robot.commands.SwerveCommands.DriveToPoseCommand;
import frc.robot.commands.SwerveCommands.SwerveJoystickCmd;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {

  private final CameraSubsystem cameraSubsystem = new CameraSubsystem();
  private final ColorSensor sensor = new ColorSensor();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(cameraSubsystem);
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(sensor);
  private final ArmSubsystem faker = new ArmSubsystem();
  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private final AngleCalculator calculator = new AngleCalculator(swerveSubsystem);
  private final IntakeSubsystem intake = new IntakeSubsystem(sensor);
  private final Joystick emergencyJoystick = new Joystick(1);
  private final AutoRoutine1 autoRoutine1 = new AutoRoutine1(swerveSubsystem, cameraSubsystem, shooterSubsystem, intake, faker, calculator, sensor);
  private final AutoRoutine2 autoRoutine2 = new AutoRoutine2(swerveSubsystem, cameraSubsystem, shooterSubsystem, intake, faker, calculator, sensor);
  private final AutoRoutine12 autoRoutine12 = new AutoRoutine12(swerveSubsystem, cameraSubsystem, shooterSubsystem, intake, faker, calculator, sensor);
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final besdakkatest ampRoutine = new besdakkatest(swerveSubsystem, cameraSubsystem, shooterSubsystem, intake, faker, calculator, sensor);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private double offSet = 0.0;

  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, 
    () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis), //buna bak
    () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
    () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
    //() -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
    () -> driverJoystick.getRawAxis(3),
    () -> driverJoystick.getRawButton(OIConstants.kDriverNoteButtonIdx)));
    configureBindings();

    m_chooser.addOption("uzak oto routine", autoRoutine2);
    m_chooser.addOption("yakin garanti oto routine", autoRoutine12);
    m_chooser.setDefaultOption("yakin oto routine", autoRoutine1);

    SmartDashboard.putBoolean("button", driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx));   
    SmartDashboard.putData(m_chooser);
  }

  private void configureBindings() {

    new JoystickButton(driverJoystick, 1).onTrue(new InstantCommand(() -> {
      Constants.DahaIyiField.fieldFront = new Rotation2d(
      swerveSubsystem.getRotation2d().getRadians() + DahaIyiField.fieldFront.getRadians());
      swerveSubsystem.resetOrientation();}));
    //new JoystickButton(driverJoystick, 2).whileTrue(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.2, 4.3, 30).andThen(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.6, 5.1, 0)).andThen(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 13.8, 5.1, 0)).andThen(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.6, 5.1, 0)).andThen(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 13.8, 6.9, -30)).andThen(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.6, 5.1, 0))).onFalse(new InstantCommand(() -> swerveSubsystem.stopModules()));
    
    //new JoystickButton(driverJoystick, 2).whileTrue(new RunCommand(() -> shooterSubsystem.setShooterSpeed(0.8), shooterSubsystem)).onFalse(new InstantCommand(() -> shooterSubsystem.setShooterVoltage(0.0), shooterSubsystem));
    new JoystickButton(driverJoystick, 3).whileTrue(new RunCommand(() -> intake.setIntakeVoltage(10.0), intake)).onFalse(new InstantCommand(() -> intake.setIntakeVoltage(0.0), intake));
    new JoystickButton(driverJoystick, 5).whileTrue(new RunCommand(() -> faker.setLength(calculator.calculateIntakeAngle(DahaIyiField.speaker)), faker)).onFalse(new InstantCommand(() -> faker.setOrtaVoltage(0.0), faker));
    new JoystickButton(driverJoystick, 6).whileTrue(new RunCommand(() -> faker.setLength(0), faker)).onFalse(new InstantCommand(() -> faker.setOrtaVoltage(0.0), faker));
    new JoystickButton(driverJoystick, 7).onTrue(new grabNote(intake, shooterSubsystem));
    new JoystickButton(driverJoystick, 8).whileTrue(new RunCommand(() -> faker.setLength(21.6), faker)).onFalse(new InstantCommand(() -> faker.setOrtaVoltage(0.0), faker));
    new JoystickButton(driverJoystick, 2).whileTrue(ampRoutine).onFalse(new InstantCommand(() -> swerveSubsystem.stopModules(), swerveSubsystem));

    new Trigger(() -> driverJoystick.getRawAxis(3) > 0.5).whileTrue(new AutoShootCommand(shooterSubsystem, calculator)).onFalse(new InstantCommand(() -> shooterSubsystem.setShooterVoltage(0.0)));
    new POVButton(driverJoystick, 0).whileTrue(new RunCommand(() -> faker.moveArmPid(0.9), faker)).onFalse(new InstantCommand(() -> faker.setArmVoltage(0.0), faker));
    new POVButton(driverJoystick, 180).whileTrue((new CloseArmCommand(faker))).onFalse(new InstantCommand(() -> faker.setArmVoltage(0.0), faker));
    new POVButton(driverJoystick, 270).whileTrue(new RunCommand(() -> faker.moveArmPid(0.8), faker)).onFalse(new InstantCommand(() -> faker.setArmVoltage(0.0), faker));

    new JoystickButton(emergencyJoystick, 5).whileTrue(new RunCommand(() -> faker.setOrtaVoltage(4.0), faker)).onFalse(new InstantCommand(() -> faker.setOrtaVoltage(0.0)));
    new JoystickButton(emergencyJoystick, 6).whileTrue(new RunCommand(() -> faker.setOrtaVoltage(-4.0), faker)).onFalse(new InstantCommand(() -> faker.setOrtaVoltage(0.0)));
    //new JoystickButton(emergencyJoystick, 4).whileTrue(new RunCommand(() -> intake.setIntakeVoltage(-7.0f), intake)).onFalse(new InstantCommand(() -> intake.setIntakeVoltage(0.0), intake));
    
    new JoystickButton(emergencyJoystick, 9).onTrue(new InstantCommand(() -> faker.resetArmLengthEncoder()));
    new JoystickButton(emergencyJoystick, 4).whileTrue(new RunCommand(() -> intake.setIntakeVoltage(-6), intake)).onFalse(new InstantCommand(() -> intake.setIntakeVoltage(0), intake));
    //new JoystickButton(emergencyJoystick, 10).onTrue(new InstantCommand(() -> faker.reset))
    //new JoystickButton(emergencyJoystick, 10).whileTrue(new RunCommand(() -> faker.moveArmPid(1.7), faker)).onFalse(new RunCommand(() -> faker.moveArmPid(0.2), faker).withTimeout(1.2));


    new JoystickButton(emergencyJoystick, 7).whileTrue(new RunCommand(() -> climbSubsystem.setClimbVolts(6), climbSubsystem)).onFalse(new InstantCommand(() -> climbSubsystem.zeroMotors(), climbSubsystem));
    new JoystickButton(emergencyJoystick, 8).whileTrue(new RunCommand(() -> climbSubsystem.setClimbVolts(-6), climbSubsystem)).onFalse(new InstantCommand(() -> climbSubsystem.zeroMotors(), climbSubsystem));
    
    //new JoystickButton(emergencyJoystick, 8).whileTrue(new RunCommand(() -> climbSubsystem.moveServo(1), climbSubsystem, climbSubsystem)).onFalse(new InstantCommand(() -> climbSubsystem.moveServo(0), climbSubsystem));
  }

  public Command getAutonomousCommand() {
/* 
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);
    
     
    Trajectory trajectoryone = new Trajectory();
    String trajectoryJSON = new String("output/output/Unnamed_0.wpilib.json");
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectoryone = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
   }
*/
    
    //Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(), new Pose2d(3.0,0.0, new Rotation2d(0)), config);
    // Trajectory trajectory =  TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new Rotation2d(0)), List.of(), new Pose2d(-1,0, new Rotation2d(0)), config);

    /* 
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectoryone, swerveSubsystem::getEstimatedPose, DriveConstants.kDriveKinematics, xController, yController, thetaController,swerveSubsystem::desiredRot
    ,swerveSubsystem::setModuleStates, swerveSubsystem);

*/
    //swerveSubsystem.resetOdometry(trajectory.getInitialPose());
    
    //return new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.2, 5.5, 0).andThen(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.2, 0, 0)).andThen(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.3, 6.2, 94));
    //return new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 13.0, 5.5, 0).andThen(swerveControllerCommand.andThen(swerveControllerCommandtwo).andThen(swerveControllerCommandthree).andThen(swerveControllerCommandfour).andThen(new InstantCommand(() -> swerveSubsystem.stopModules(), swerveSubsystem)));
    //return new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.2, 5.5, 0).andThen(new InstantCommand(() -> swerveSubsystem.stopModules()));
    return (m_chooser.getSelected());
  }
}
