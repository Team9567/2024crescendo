// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

 // @Override
  //public void robotInit() {
    //m_robotContainer = new RobotContainer();
  //}

    /** This function is run once each time the robot enters autonomous mode. */
    @Override
  public void autonomousInit() {
    Object autonomousCommand = m_robotContainer.getAutonomousCommand();

  
      // schedule the autonomous command (example)
      if (autonomousCommand != null) {
        ((Command) autonomousCommand).schedule();
      }
    }
  
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

/** This function is called once each time the robot enters teleoperated mode. */
@Override
public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

/** This function is called periodically during teleoperated mode. */
@Override
public void teleopPeriodic() {

}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}


  @Override
  public void autonomousExit() {}

  @Override
  public void teleopExit() {}

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

/** This function is called periodically during test mode. */
@Override
public void testPeriodic() {}

  @Override
  public void testExit() {}
}
