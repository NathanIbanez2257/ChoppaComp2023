// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.limelightCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drive;

import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final drive driveSub = new drive();
  private final PS4Controller nathan = new PS4Controller(DriveConstants.nathan);


  private final limelightCommand aiming = new limelightCommand(driveSub, DriveConstants.targetDistance);

  String path1 = "curvePath.wpilib.json";

  String slow = "normalPath.wpilib.json";

  String slowest = "slowestPath.wpilib.json";

  SendableChooser<Command> chooser = new SendableChooser<>();

  RunCommand nathanMove = new RunCommand(
      () -> driveSub.move(DriveConstants.driveSpeed * nathan.getRawAxis(DriveConstants.leftDriveAxis),
          DriveConstants.driveSpeed * nathan.getRawAxis(DriveConstants.rightDriveAxis)),
      driveSub);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(DriveConstants.nathan);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driveSub.setDefaultCommand(nathanMove);

    chooser.addOption("curvePath",
        loadPathPlannerTrajectoryToRamseteCommand(
            path1, true));

    chooser.addOption("slowPath",
        loadPathPlannerTrajectoryToRamseteCommand(
            slow, true));

    Shuffleboard.getTab("Autonomous").add(chooser);

      JoystickButton aimLimelight = new JoystickButton(nathan, ButtonConstants.limelightButton);
      aimLimelight.whileTrue(aiming);

    // Configure the trigger bindings
    configureBindings();
  }

  
  /**
   * example commment
   * 
   * @param filename
   * @param resetOdometry
   * @return
   */

  public Command loadPathPlannerTrajectoryToRamseteCommand(String filename, boolean resetOdometry) {
    Trajectory trajectory;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      System.out.println("winning");
    }

    catch (Exception exception) {
      DriverStation.reportError("Unable To Open Trajectory " + filename, exception.getStackTrace());
      System.out.println("Unable to read from file" + filename);
      return new InstantCommand();
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, driveSub::getPose,
        new RamseteController(DriveConstants.kRamsesteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        driveSub::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVelcotiy, 0, 0),
        new PIDController(DriveConstants.kPDriveVelcotiy, 0, 0),
        driveSub::tankDriveVolts,
        driveSub);

    if (resetOdometry) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> driveSub.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
    }

    else {
      return ramseteCommand;
    }

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return chooser.getSelected();

    //return Autos.exampleAuto(m_exampleSubsystem);
  }
}
