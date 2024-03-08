// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class RobotLauncher extends SubsystemBase {
  CANSparkMax m_launchWheel;
  CANSparkMax m_feedWheel;

  /** Creates a new Launcher. */
  public RobotLauncher() {
    m_launchWheel = new CANSparkMax(LauncherConstants.kLauncherID, MotorType.kBrushless);
    m_feedWheel = new CANSparkMax(LauncherConstants.kFeederID, MotorType.kBrushless);

    m_launchWheel.setSmartCurrentLimit(LauncherConstants.kLauncherCurrentLimit);
    m_feedWheel.setSmartCurrentLimit(LauncherConstants.kFeedCurrentLimit);

    m_launchWheel.setInverted(false);
    m_feedWheel.setInverted(false);

    m_launchWheel.clearFaults();
    m_launchWheel.setIdleMode(LauncherConstants.kLaunchBrakeMode);
    m_feedWheel.clearFaults();
    m_feedWheel.setIdleMode(LauncherConstants.kFeedBrakeMode);

  }

  /**
   * This method is an example of the 'subsystem factory' style of command
   * creation. A method inside
   * the subsytem is created to return an instance of a command. This works for
   * commands that
   * operate on only that subsystem, a similar approach can be done in
   * RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as
   * the startEnd
   * method used here, to create these commands.
   */
  public Command getIntakeCommand() {
    // The startEnd helper method takes a method to call when the command is
    // initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setFeedWheel(LauncherConstants.kIntakeFeederSpeed);
          setLaunchWheel(LauncherConstants.kIntakeLauncherSpeed);
        },        
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }

    public Command ampLauncher() {
  
    return this.startEnd(

        () -> {
          setFeedWheel(LauncherConstants.kAmpLaunchSpeed);
          setLaunchWheel(LauncherConstants.kAmpLaunchSpeed);
        },        

        () -> {
          stop();
        });
  }

  // An accessor method to set the speed (technically the output percentage) of
  // the launch wheel
  public void setLaunchWheel(double speed) {
    m_launchWheel.set(speed);
  }

  // An accessor method to set the speed (technically the output percentage) of
  // the feed wheel
  public void setFeedWheel(double speed) {
    m_feedWheel.set(speed);
  }

  // A helper method to stop both wheels. You could skip having a method like this
  // and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    m_launchWheel.set(0);
    m_feedWheel.set(0);
  }
}
