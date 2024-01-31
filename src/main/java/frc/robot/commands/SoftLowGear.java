// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.autonomousCommand;
import edu.wpi.first.wpilibj2.command.Command;

public class SoftLowGear extends Command {
  boolean gear = false;
  // Make a class command called soft low gear. Include a function that returns it
  // to false

  public Command setGear() {
    // The startEnd helper method takes a method to call when the command is
    // initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          // start section
          // set low gear
          setLowGear();
          
        },
        () -> {
        //stop section
        // set high gear
        setHighGear();
        });
  }

  public setLowGear() {
    gear = true;
  }

  public setHighGear() {
    gear = false;
  }
}