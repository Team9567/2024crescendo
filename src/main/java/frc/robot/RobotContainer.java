// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotChassisConstants;
import frc.robot.Constants.autonomousCommand;
import frc.robot.commands.IntakeNoteWithShooter;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
import frc.robot.subsystems.RobotChassis;
import frc.robot.subsystems.RobotClimber;
import frc.robot.subsystems.RobotLauncher;
import frc.robot.subsystems.UnderTheBumperGroundIntake;
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
  public UnderTheBumperGroundIntake intakeMotor = new UnderTheBumperGroundIntake();

  // ROBOT COMMAND DEFINITIONS

  // JOYSTICK AND BUTTON ASSIGNMENTS
  public CommandJoystick driver = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  public CommandJoystick controller = new CommandJoystick(OperatorConstants.kOperatorControllerPort);

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  SendableChooser<Command> colorChooser = new SendableChooser<>();
  public double sleepTimeout = 0.0;

  // The container for the robot. Contains subsystems, OI devices, and commands
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    autoChooser.addOption("Middle", shootAndReatreat(1, 0, 1));

    autoChooser.addOption("SHOOT ONLY", shootAndReatreat(0, 0, 0));
    autoChooser.addOption("back up turn ", shootAndReatreat(0.4, 30, 0.5));

    /*
     * 
     * 
     * 
     * 
     * 
     * /*
     * // autoChooser.setDefaultOption("shoot and retreat", shootAndReatreat());
     * autoChooser.addOption("BlueShort",
     * shootAndReatreat(autonomousCommand.kBlueShortBack1,
     * autonomousCommand.kBlueShortTurnDistance2, autonomousCommand.kBlueShortBack2,
     * autonomousCommand.KCounterClockWise));
     * /*
     * autoChooser.addOption("posB", shootAndReatreat(autonomousCommand.kPosBTurn1,
     * autonomousCommand.kPosBBack1,
     * autonomousCommand.kPosBTurn2, autonomousCommand.kPosBBack2));
     * 
     * autoChooser.addOption("BlueLong",
     * shootAndReatreat(autonomousCommand.kBlueLongBack1,
     * autonomousCommand.kBlueLongTurnDistance2, autonomousCommand.kBlueLongBack2,
     * autonomousCommand.kClockWise));
     * 
     * 
     * autoChooser.addOption("RedShort",
     * shootAndReatreat(autonomousCommand.kRedShortBack1,
     * autonomousCommand.kRedShortTurnDistance2, autonomousCommand.kRedShortBack2,
     * autonomousCommand.kClockWise));
     * 
     * autoChooser.addOption("posB", shootAndReatreat(autonomousCommand.kPosBTurn1,
     * autonomousCommand.kPosBBack1,
     * autonomousCommand.kPosBTurn2, autonomousCommand.kPosBBack2));
     * 
     * autoChooser.addOption("RedLong",
     * shootAndReatreat(autonomousCommand.kRedLongBack1,
     * autonomousCommand.kRedLongTurnDistance2, autonomousCommand.kRedLongBack2,
     * autonomousCommand.KCounterClockWise));
     */
    SmartDashboard.putData("AutoPosition", autoChooser);

    SmartDashboard.putNumber("AutoWaitTime", sleepTimeout);

  }

  public void configureBindings() {
    chassis.setDefaultCommand(
        new RunCommand(
            () -> {
              chassis.arcadeDrive(driver.getRawAxis(1), driver.getRawAxis(0));
            }, chassis));
    // attach drive distance to button A11
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
     * Create an inline sequence to run when the operator presses the A
     * (green) button. Run the PrepareLaunch
     * command for 1 seconds and then run the LaunchNote command
     */

    ParallelCommandGroup intakeAndLaunchGroup = new ParallelCommandGroup();
    // intakeAndLaunchGroup.beforeStarting(new
    // PrepareLaunch(launcher).withTimeout(LauncherConstants.kLauncherDelay));
    intakeAndLaunchGroup.addCommands(new LaunchNote(launcher), intakeMotor.runGroundForShoot());
    // SequentialCommandGroup launchNote = new SequentialCommandGroup(new
    // PrepareLaunch(launcher).withTimeout(LauncherConstants.kLauncherDelay),
    // intakeAndLaunchGroup);
    controller
        .button(OperatorConstants.kOperatorButtonLaunch)
        .onTrue(
            new PrepareLaunch(launcher).withTimeout(LauncherConstants.kLauncherDelay)
                .andThen(new LaunchNote(launcher).withTimeout(3)));
    /*
     * controller
     * .button(OperatorConstants.kOperatorButtonLaunch)
     * .onTrue(
     * launchNote.withTimeout(5)
     * );
     */

    /*
     * new PrepareLaunch(launcher)
     * .withTimeout(LauncherConstants.kLauncherDelay)
     * .alongWith(new
     * LaunchNote(launcher).withTimeout(OperatorConstants.klauncherRunTimeConstant),
     * intakeMotor.runGroundForShoot()));
     * //.andThen(new
     * LaunchNote(launcher).withTimeout(OperatorConstants.klauncherRunTimeConstant))
     * );
     */

    driver
        .button(OperatorConstants.kDriveOrientApriltag).whileTrue(
            vision.getOrientAprilTag(chassis));

    // Set up a binding to run the intake command while the operator is pressing and
    // holding the
    // left Bumper

    controller.button(OperatorConstants.kOperatorButtonIntake).whileTrue(launcher.getIntakeCommand());

    controller.button(OperatorConstants.kOperatorButtonAmp).whileTrue(launcher.ampLauncher());

    // Buttons for operating the ground intake
    // controller.button(6).whileTrue(intakeMotor.groundIntaking().until(()->
    // intakeMotor.intakeBlocked()));
    Command intakeGroup = makeGroundIntakeCommand();
    controller.button(6).onTrue(
        intakeGroup.until(() -> intakeMotor.intakeBlocked()).andThen(launcher.getIntakeCommand().withTimeout(5)));
    controller.button(7).onTrue(intakeMotor.groundExtacking().withTimeout(.5));

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

  public Command makeGroundIntakeCommand() {
    ParallelCommandGroup intakeGroup = new ParallelCommandGroup();
    intakeGroup.addCommands(intakeMotor.groundIntaking(), new IntakeNoteWithShooter(launcher));
    return intakeGroup;
  }

  public Command shootAndReatreat(double retreat1, double rotate1, double retreat2) {
    // do we want a negation here? is this too much complexity and confusing????
    // NateO - 10:36AM 3/2/24

    navxGyro.reset();

    double sleepTimer = SmartDashboard.getNumber("AutoWaitTime", sleepTimeout);

    return new PrepareLaunch(launcher)
        // launches for delay + 2.5 seconds
        .withTimeout(LauncherConstants.kLauncherDelay)
        .andThen(new LaunchNote(launcher)
            .withTimeout(2.5))
        // wait in place for sleepTimer
        /*
         * .andThen(new RunCommand(
         * () -> {
         * }).withTimeout(sleepTimer))
         */
        // retreat towards wall
        .andThen(new RunCommand(
            () -> {
              chassis.chassisToBearing(rotate1 * -1);
            }, chassis)
            .withTimeout(3))
        .andThen(
            new ParallelCommandGroup(
                makeGroundIntakeCommand().until(() -> intakeMotor.intakeBlocked()).andThen(launcher.getIntakeCommand())
                    .withTimeout(2),
                new RunCommand(
                    () -> {
                      chassis.arcadeDrive(0.5 * 1, 0);
                    }, chassis)
                    .withTimeout(retreat1)))
        // turn parallel to wall

        // drive parallel to wall
        .andThen(new RunCommand(
            () -> {
              chassis.arcadeDrive(0.5 * -1, 0); // do we want a negation here? is this too much complexity and
                                                // confusing???? NateO - 10:36AM 3/2/24
            }, chassis)
            .withTimeout(retreat2))
        .andThen(new RunCommand(
            () -> {
              chassis.chassisToBearing(rotate1 * 1);
            }, chassis)
            .withTimeout(3))
        .andThen(new PrepareLaunch(launcher)
            // launches for delay + 2.5 seconds
            .withTimeout(LauncherConstants.kLauncherDelay)
            .andThen(new LaunchNote(launcher)
                .withTimeout(2.5)));

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
