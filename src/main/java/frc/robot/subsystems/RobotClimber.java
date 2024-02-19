package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotClimberConstants;

public class RobotClimber extends SubsystemBase{
    public CANSparkMax m_climberRight = new CANSparkMax(RobotClimberConstants.kClimberRightID, MotorType.kBrushless);
    public RelativeEncoder m_climberRightEncoder = m_climberRight.getEncoder();
    CANSparkMax m_climberLeft;
    //targeting at 15 degrees we get 0.16 power at 45 we get 0.5, we half it for each motor
    PIDController thetaController = new PIDController(1 / 45, 0, 0); // fix constants
    AHRS navxGyro;
    
    public RobotClimber(AHRS gyro) {

        this.navxGyro = gyro;

        m_climberLeft = new CANSparkMax(RobotClimberConstants.kClimberLeftID, MotorType.kBrushless);

        m_climberRight.clearFaults();
        m_climberRight.setIdleMode(IdleMode.kBrake);
        m_climberLeft.clearFaults();
        m_climberLeft.setIdleMode(IdleMode.kBrake);
        m_climberRight.setInverted(RobotClimberConstants.kClimberRightInversion);
        m_climberLeft.setInverted(RobotClimberConstants.kClimberLeftInversion);
    }


    public void leftClimb(double power){

        m_climberLeft.set(power);

    }

    public void rightClimb(double power){

        m_climberRight.set(power);

    }

    public void autoClimb(double power) {

        double initalBearing = navxGyro.getRoll();
        double output = thetaController.calculate(initalBearing, 0);

        rightClimb(power + output/2);
        leftClimb(power - output/2);

    }



}

