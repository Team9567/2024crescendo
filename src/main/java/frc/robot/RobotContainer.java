// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotChassisConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.PrepareLaunch;
import frc.robot.commands.LaunchNote;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RobotChassis;
import frc.robot.subsystems.RobotLauncher;
import frc.robot.subsystems.Vision;

public class RobotContainer {

  public Field2d field = new Field2d();

  public AHRS navxGyro = new AHRS(edu.wpi.first.wpilibj.I2C.Port.kMXP);

  public DifferentialDriveKinematics differentialDriveKinematics = new DifferentialDriveKinematics(RobotChassisConstants.kTrackWidth);

  public DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(differentialDriveKinematics,
      navxGyro.getRotation2d(), 0, 0, new Pose2d());
  // rotation2d not always 0

  // SUBSYSTEMS

  public RobotChassis chassis = new RobotChassis(navxGyro, poseEstimator, field);
  public RobotLauncher launcher = new RobotLauncher();
  public Vision vision = new Vision(poseEstimator, field);

  // ROBOT COMMAND DEFINITIONS

  // JOYSTICK AND BUTTON ASSIGNMENTS
  public CommandJoystick driver = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  public CommandJoystick controller = new CommandJoystick(OperatorConstants.kOperatorControllerPort);

  // The container for the robot. Contains subsystems, OI devices, and commands
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  public void configureBindings() {
    chassis.setDefaultCommand(
        new RunCommand(
            () -> {
              chassis.arcadeDrive(-driver.getRawAxis(1), driver.getRawAxis(0));
            }, chassis));
    // attach drive distance to button A
    // m_Chooser.addOption("drive 5 feet", new
    // DriveDistanceCommand(RobotChassis.class));
    /*
     * Create an inline sequence to run when the operator presses and holds the A
     * (green) button. Run the PrepareLaunch
     * command for 1 seconds and then run the LaunchNote command
     */
    controller
        .button(OperatorConstants.kOperatorButtonLaunch)
        .whileTrue(
            new PrepareLaunch(launcher)
                .withTimeout(LauncherConstants.kLauncherDelay)
                .andThen(new LaunchNote(launcher))
                .handleInterrupt(() -> launcher.stop()));

    // Set up for the binding for the soft low gear
    driver.button(OperatorConstants.kDriverButtonGear).onTrue(new InstantCommand(() -> {
      chassis.setLowGear();
    }));
    driver.button(OperatorConstants.kDriverButtonGear).onFalse(new InstantCommand(() -> {
      chassis.setHighGear();
    }));

    // Set up a binding to run the intake command while the operator is pressing and
    // holding the
    // left Bumper
    controller.button(OperatorConstants.kOperatorButtonIntake).whileTrue(launcher.getIntakeCommand());

    // public Command getAutonomousCommand() {
    //  return new DriveDistanceCommand(chassis);
    //}
  }
}

//COntroller
//Change the button for shooting to press
//Shoot B - current

//Source intake, X - current

//Floor intake, press button hold down, pick up ring, when picked up, retract, button A, let her have the option to retrack mannually store position
//Intake button, move down, hold to move motors on the intake motors to move


//One button for ready position A stays within frame perameter ready for amp out of frame perimeter then back to Amp
//Store low, no note button A
//Store high, stores high Button A


//place in AMP, button Y, intake takeover with button push. From the High position

//Leave climber in the open, left and right trigger. Maybe front two bumpers.

//Left, right, middle climb button.
//Joysticks for manually moving climber arms
//Up down on both acuators move bolth independetly on climber

//Driver
//Orient button - A, chose an area with the nav x, position a direction, EX: Push a button turn to the left, towards the other allience, look towards the opposite side

//Lime light button - Y, Orients and points at an april tag.

//Switch the back and the front High end front

//180 on AMP april tag?
