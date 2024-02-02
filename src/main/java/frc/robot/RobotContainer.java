// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.PrepareLaunch;
import frc.robot.commands.LaunchNote;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.RobotChassis;
import frc.robot.subsystems.RobotLauncher;
import frc.robot.subsystems.Vision;


public class RobotContainer {

  public Field2d field = new Field2d();

  public AHRS navxGyro = new AHRS(edu.wpi.first.wpilibj.I2C.Port.kMXP); // port might be wrong

  public DifferentialDriveKinematics differentialDriveKinematics = new DifferentialDriveKinematics(1); // change later

  public DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(differentialDriveKinematics, navxGyro.getRotation2d(), 0, 0, new Pose2d());
  //rotation2d not always 0

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

    // Set up a binding to run the intake command while the operator is pressing and
    // holding the
    // left Bumper
    driver.button(2).whileTrue(launcher.getIntakeCommand());
    //Set up for the binding for the soft low gear
    driver.button(3).onTrue(new InstantCommand(()->{chassis.setLowGear();}));
    driver.button(3).onFalse(new InstantCommand(()->{chassis.setHighGear();}));
  // public Command getAutonomousCommand() {
  // return new DriveDistanceCommand(chassis);
  }
}
