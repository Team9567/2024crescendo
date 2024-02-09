// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

public class StoreNote extends Command {

  

  //Arm swings around a fixed pivot using one motor, intake uses two motors on a belt to intake a ring detected by a graple sensor

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Detect if a ring is in the arm
    //If there is a ring in the in the intake, set a variable to true that will toggle in execute to move the arm up
    //else set the variable to false
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //If the variable has returned true, meaning there is a note in the arm, swing the arm upwards. 
    /*Else the variable has returned false, meaning there is no note in the arm. Swing the arm downwards. 
    *While a note is not detected by the laser can sensor in the arm and the button hasn't been released, 
    *spin the intake wheels. Once the laser has detected a note, return the variable as false and restart the command, swinging the arm upwards.
    *Make sure the code can be cancelled at any time by releasing the button.
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Do nothing when the command ends. The launch wheel needs to keep spinning in order to launch
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use a timeout
    // decorator on the command to end it.
    return false;
  }
}