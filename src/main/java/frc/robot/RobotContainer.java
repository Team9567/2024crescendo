// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotChassisConstants;
import frc.robot.Constants.autonomousCommand;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
import frc.robot.subsystems.RobotChassis;
import frc.robot.subsystems.RobotClimber;
import frc.robot.subsystems.RobotLauncher;
import frc.robot.subsystems.Vision;

public class RobotContainer {

  public Field2d field = new Field2d();

  public AHRS navxGyro = new AHRS(edu.wpi.first.wpilibj.I2C.Port.kMXP);

  public DifferentialDriveKinematics differentialDriveKinematics = new DifferentialDriveKinematics(
      RobotChassisConstants.kTrackWidth);

  public DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(differentialDriveKinematics,
      navxGyro.getRotation2d(), 0, 0, new Pose2d());
  // rotation2d not always 0

  // SUBSYSTEMS

  public RobotChassis chassis = new RobotChassis(navxGyro, poseEstimator, field);
  public RobotLauncher launcher = new RobotLauncher();
  public Vision vision = new Vision(poseEstimator, field);
  public RobotClimber climber = new RobotClimber(navxGyro);

  // ROBOT COMMAND DEFINITIONS

  // JOYSTICK AND BUTTON ASSIGNMENTS
  public CommandJoystick driver = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  public CommandJoystick controller = new CommandJoystick(OperatorConstants.kOperatorControllerPort);

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  public double sleepTimeout = 0.0;

  // The container for the robot. Contains subsystems, OI devices, and commands
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // autoChooser.setDefaultOption("shoot and retreat", shootAndReatreat());
    autoChooser.addOption("posA", shootAndReatreat(autonomousCommand.kPosATurn1, autonomousCommand.kPosABack1,
        autonomousCommand.kPosATurn2, autonomousCommand.kPosABack2));
    autoChooser.addOption("posB", shootAndReatreat(autonomousCommand.kPosBTurn1, autonomousCommand.kPosBBack1,
        autonomousCommand.kPosBTurn2, autonomousCommand.kPosBBack2));
    autoChooser.addOption("posC", shootAndReatreat(autonomousCommand.kPosCTurn1, autonomousCommand.kPosCBack1,
        autonomousCommand.kPosCTurn2, autonomousCommand.kPosCBack2));

    SmartDashboard.putData("AutoPosition", autoChooser);

    SmartDashboard.putNumber("AutoWaitTime", sleepTimeout);

  }

  public void configureBindings() {
    chassis.setDefaultCommand(
        new RunCommand(
            () -> {
              chassis.arcadeDrive(driver.getRawAxis(1), driver.getRawAxis(0));
            }, chassis));
    // attach drive distance to button A
    // m_Chooser.addOption("drive 5 feet", new
    // DriveDistanceCommand(RobotChassis.class));

    // Set up for the binding for the soft low gear
    driver.button(OperatorConstants.kDriverButtonGear).onTrue(new InstantCommand(() -> {
      chassis.setLowGear();
    }));
    driver.button(OperatorConstants.kDriverButtonGear).onFalse(new InstantCommand(() -> {
      chassis.setHighGear();
    }));
    /*
     * Create an inline sequence to run when the operator presses and holds the A
     * (green) button. Run the PrepareLaunch
     * command for 1 seconds and then run the LaunchNote command
     */
    controller
        .button(OperatorConstants.kOperatorButtonLaunch)
        .onTrue(
            new PrepareLaunch(launcher)
                .withTimeout(LauncherConstants.kLauncherDelay)
                .andThen(new LaunchNote(launcher).withTimeout(OperatorConstants.klauncherRunTimeConstant)));

    // Set up a binding to run the intake command while the operator is pressing and
    // holding the
    // left Bumper

    controller.button(OperatorConstants.kOperatorButtonIntake).whileTrue(launcher.getIntakeCommand());

    //AMP NOTE SCORE 10 percent power puke
    //controller.button(OperatorConstants.kOperatorButtonAmp).whileTrue(launcher.ampLauncher());

    controller

        .axisGreaterThan(OperatorConstants.kOperatorAxisLeftClimb, 0.05)
        .or(controller.axisLessThan(OperatorConstants.kOperatorAxisLeftClimb, -0.05))
        .or(controller.axisGreaterThan(OperatorConstants.kOperatorAxisRightClimb, 0.05))
        .or(controller.axisLessThan(OperatorConstants.kOperatorAxisRightClimb, -0.05))
        .whileTrue(
            new RunCommand(
                () -> {
                  climber.leftClimb(controller.getRawAxis(OperatorConstants.kOperatorAxisLeftClimb));
                  climber.rightClimb(controller.getRawAxis(OperatorConstants.kOperatorAxisRightClimb));
                }, climber)

                .finallyDo(
                    () -> {
                      climber.leftClimb(0);
                      climber.rightClimb(0);

                    }));

    // public Command getAutonomousCommand() {
    // return new DriveDistanceCommand(chassis);
    // }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command shootAndReatreat(double rotate1, double retreat1, double rotate2, double retreat2) {

    double sleepTimer = SmartDashboard.getNumber("AutoWaitTime", sleepTimeout);
    Optional<Alliance> ally = DriverStation.getAlliance();
    double allianceTurnDirection = 1;
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        allianceTurnDirection = -1;
      } else if (ally.get() == Alliance.Blue) {
        allianceTurnDirection = 1;
      }
    }

    final double turn = allianceTurnDirection;

    return new PrepareLaunch(launcher)
        // launches for delay + 2.5 seconds
        .withTimeout(LauncherConstants.kLauncherDelay)
        .andThen(new LaunchNote(launcher)
            .withTimeout(2.5))
        // wait in place for sleepTimer
        .andThen(new RunCommand(
            () -> {
            }).withTimeout(sleepTimer))
        // turn if necessary
        .andThen(new RunCommand(
            () -> {
              chassis.arcadeDrive(0, 0.5 * turn);
            }, chassis)
            .withTimeout(rotate1))
        // retreat towards wall
        .andThen(new RunCommand(
            () -> {
              chassis.arcadeDrive(0.5, 0);
            }, chassis)
            .withTimeout(retreat1))
        // turn parallel to wall
        .andThen(new RunCommand(
            () -> {
              chassis.arcadeDrive(0, 0.5 * turn);
            }, chassis)
            .withTimeout(rotate2))
        // drive parallel to wall
        .andThen(new RunCommand(
            () -> {
              chassis.arcadeDrive(0.5, 0);
            }, chassis)
            .withTimeout(retreat2));

  }

}
// Controller
// Shoot B - current

// Source intake, X - current

// Floor intake, press button hold down, pick up ring, when picked up, retract,
// button A, let her have the option to retrack mannually store position
// Intake button, move down, hold to move motors on the intake motors to move

// One button for ready position A stays within frame perameter ready for amp
// out of frame perimeter then back to Amp
// Store low, no note button A
// Store high, stores high Button A

// place in AMP, button Y, intake takeover with button push. From the High
// position

// Leave climber in the open, left and right trigger. Maybe front two bumpers.

// Left, right, middle climb button.
// Joysticks for manually moving climber arms
// Up down on both acuators move bolth independetly on climber

// Driver
// Orient button - A, chose an area with the nav x, position a direction, EX:
// Push a button turn to the left, towards the other allience, look towards the
// opposite side

// Lime light button - Y, Orients and points at an april tag.

// 180 on AMP april tag?
