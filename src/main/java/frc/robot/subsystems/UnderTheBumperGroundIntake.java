
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Distance sensor, grappleHook, time of flight(TOF)

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UnderTheBumperConstants;

public class UnderTheBumperGroundIntake extends SubsystemBase {
  public CANSparkMax underTheBumperMotor = new CANSparkMax(UnderTheBumperConstants.kUnderTheBumperCanId,
      MotorType.kBrushless);
  public CANSparkMax underTheBumperFollower = new CANSparkMax(UnderTheBumperConstants.kUnderTheBumperFollowerID,
      MotorType.kBrushless);
  public LaserCan laserCan = new LaserCan(9);
  public boolean noteStored = false;
  // laserC
  public UnderTheBumperGroundIntake(){
    try {
    laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e){
      System.out.println("LaserCan error " + e);
    }
  }

  public void groundIntake() {
    // sets the defaults on the motor
    underTheBumperMotor.clearFaults();
    underTheBumperMotor.restoreFactoryDefaults();
    underTheBumperMotor.setIdleMode(IdleMode.kBrake);
    underTheBumperMotor.setSmartCurrentLimit(60);
    underTheBumperFollower.clearFaults();
    underTheBumperFollower.restoreFactoryDefaults();
    underTheBumperFollower.setIdleMode(IdleMode.kBrake);
    underTheBumperFollower.setSmartCurrentLimit(60);

    // Sets follower
    // underTheBumperFollower.follow(underTheBumperMotor);

    // SEts inversion
    underTheBumperMotor.setInverted(false);
    underTheBumperFollower.setInverted(true);
  }

  public void runGroundIntake() {
    underTheBumperMotor.set(UnderTheBumperConstants.kUnderTheBumperMotorIntakeSpeed * -1);
    underTheBumperFollower.set(UnderTheBumperConstants.kUnderTheBumperMotorIntakeSpeed * -1);
  }

  public void runGroundExtake() {
    underTheBumperMotor.set(UnderTheBumperConstants.kUnderTheBumperMotorExtakeSpeed);
    underTheBumperFollower.set(UnderTheBumperConstants.kUnderTheBumperMotorExtakeSpeed);

  }

  public Command groundExtacking() {
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          runGroundExtake();
        },
        // When the command stops, stop the wheels
        () -> {
          underTheBumperMotor.set(0);
          underTheBumperFollower.set(0);
        });
  }

  public boolean intakeBlocked() {
    if (noteStored){
      return true;
    }
    LaserCan.Measurement measurement = laserCan.getMeasurement();
    int distanceMeasuremnt = measurement.distance_mm;
    SmartDashboard.putNumber("Laser Can", distanceMeasuremnt);
    if (distanceMeasuremnt < UnderTheBumperConstants.kRingDistanceMM) {
      noteStored = true;
      return true;
    } else {
      return false;
    }

  }

  public Command groundIntaking() {
    return this.runEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {

          if (this.intakeBlocked()){
            underTheBumperMotor.set(0);
            underTheBumperFollower.set(0);
          } else {
            runGroundIntake();
          }
          },
          () -> {
            underTheBumperMotor.set(0);
            underTheBumperFollower.set(0);
          }
        );
        
  }

  public Command runGroundForShoot() {
    return this.startEnd(
        () -> {
          runGroundIntake();
        },
        // When the command stops, stop the wheels
        () -> {
          underTheBumperMotor.set(0);
          underTheBumperFollower.set(0);
          noteStored = false;
        });
  }
}
