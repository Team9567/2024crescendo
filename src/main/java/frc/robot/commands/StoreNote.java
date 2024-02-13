package frc.robot.commands;

import frc.robot.Constants.GroundIntakeConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class StoreNote extends Command {

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      // There is nothing we need this command to do on each iteration. You could remove this method
      // and the default blank method
      // of the base class will run.
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