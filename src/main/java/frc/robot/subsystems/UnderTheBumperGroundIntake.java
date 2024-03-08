
package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.RobotChassisConstants;
import frc.robot.Constants.UnderTheBumperConstants;

public class UnderTheBumperGroundIntake{
    public CANSparkMax underTheBumperMotor = new CANSparkMax(UnderTheBumperConstants.kUnderTheBumperCanId, MotorType.kBrushless);

    public void groundIntake(){
        underTheBumperMotor.clearFaults();
        underTheBumperMotor.restoreFactoryDefaults();
        underTheBumperMotor.setIdleMode(IdleMode.kBrake);
        underTheBumperMotor.setSmartCurrentLimit(60);
    }

    public void runGroundIntake(){
        underTheBumperMotor.set(.5);
    }

    public void rejectNote(){
        underTheBumperMotor.set(-.5);
    }
}

