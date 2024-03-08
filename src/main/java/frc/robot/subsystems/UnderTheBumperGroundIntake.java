
package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.UnderTheBumperConstants;

public class UnderTheBumperGroundIntake extends SubsystemBase{
    public CANSparkMax underTheBumperMotor = new CANSparkMax(UnderTheBumperConstants.kUnderTheBumperCanId, MotorType.kBrushless);

    public void groundIntake(){
        underTheBumperMotor.clearFaults();
        underTheBumperMotor.restoreFactoryDefaults();
        underTheBumperMotor.setIdleMode(IdleMode.kBrake);
        underTheBumperMotor.setSmartCurrentLimit(60);
    }

    public void runGroundIntake(){
        underTheBumperMotor.set(UnderTheBumperConstants.kUnderTheBumperMotorSpeed);
    }

    public void runGroundExtake(){
        underTheBumperMotor.set(UnderTheBumperConstants.kUnderTheBumperMotorSpeed * -1);
    }

    public Command groundExtacking(){
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

    public Command groundIntaking(){
        return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          runGroundIntake();
        },        
        // When the command stops, stop the wheels
        () -> {
           underTheBumperMotor.set(0);
        });
  }
}

