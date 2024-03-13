
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import au.grapplerobotics.LaserCan;

//Distance sensor, grappleHook, time of flight(TOF)

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UnderTheBumperConstants;

public class UnderTheBumperGroundIntake extends SubsystemBase {
  public CANSparkMax underTheBumperMotor = new CANSparkMax(UnderTheBumperConstants.kUnderTheBumperCanId,
      MotorType.kBrushless);
  public CANSparkMax underTheBumperFollower = new CANSparkMax(UnderTheBumperConstants.kUnderTheBumperFollowerID,
      MotorType.kBrushless);
  public LaserCan laserCan = new LaserCan(0);
  // laserC

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
    underTheBumperFollower.follow(underTheBumperMotor);
    underTheBumperFollower.setInverted(true);
  }

  public void runGroundIntake() {
    underTheBumperMotor.set(UnderTheBumperConstants.kUnderTheBumperMotorSpeed);
  }

  public void runGroundExtake() {
    underTheBumperMotor.set(UnderTheBumperConstants.kUnderTheBumperMotorSpeed * -1);
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
        });
  }

  public Command groundIntaking() {
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          runGroundIntake();
          LaserCan.Measurement measurement = laserCan.getMeasurement();
          int distanceMeasuremnt = measurement.distance_mm;
          if (distanceMeasuremnt < UnderTheBumperConstants.kRingDistanceMM){
            underTheBumperMotor.set(0);
          }
        },        
        // When the command stops, stop the wheels
        () -> {
          underTheBumperMotor.set(0);
        });
  }

  public Command runGroundForShoot(){
    return this.startEnd(
      () -> {
          runGroundIntake();
        },        
        // When the command stops, stop the wheels
        () -> {
           underTheBumperMotor.set(0);
        });
  }
}


