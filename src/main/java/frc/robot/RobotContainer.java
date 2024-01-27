// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.PrepareLaunch;
import frc.robot.commands.LaunchNote;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.RobotChassis;
import frc.robot.subsystems.RobotLauncher;
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OperatorConstants;

public class RobotContainer {
  // SUBSYSTEMS

  public RobotChassis chassis = new RobotChassis();
  public RobotLauncher launcher = new RobotLauncher();
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
    controller.button(OperatorConstants.kOperatorButtonIntake).whileTrue(launcher.getIntakeCommand());
  }

  // public Command getAutonomousCommand() {
  // return new DriveDistanceCommand(chassis);
  // }
}
