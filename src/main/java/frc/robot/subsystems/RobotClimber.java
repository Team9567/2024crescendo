package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotClimberConstants;
import com.revrobotics.SparkPIDController;
import frc.robot.subsystems.LinearActuatorHomer;

public class RobotClimber extends SubsystemBase {
    public CANSparkMax m_climberRight;
    public CANSparkMax m_climberLeft;

    // Defineing encoders
    public RelativeEncoder rightClimberEncoder;
    public RelativeEncoder leftClimberEncoder;

    // targeting at 15 degrees we get 0.16 power at 45 we get 0.5, we half it for
    // each motor
    PIDController thetaController = new PIDController(1 / 45, 0, 0); // fix constants
    SparkPIDController m_rightPidController;
    SparkPIDController m_leftPidController;
    double pidP = RobotClimberConstants.kP; // The local variable in our code
    AHRS navxGyro;

    // LinearActuatorHomer
    LinearActuatorHomer rightHomer;
    LinearActuatorHomer leftHomer;

    public RobotClimber(AHRS gyro) {

        this.navxGyro = gyro;

        // public final CANSparkMax m_climberRight = new
        // CANSparkMax(RobotClimberConstants.kClimberLeftID, MotorType.kBrushless);
        // Motors
        m_climberRight = new CANSparkMax(RobotClimberConstants.kClimberLeftID, MotorType.kBrushless);
        m_climberLeft = new CANSparkMax(RobotClimberConstants.kClimberRightID, MotorType.kBrushless);

        m_climberRight.restoreFactoryDefaults();
        m_climberLeft.restoreFactoryDefaults();
        m_climberRight.clearFaults();
        m_climberRight.setIdleMode(IdleMode.kBrake);
        m_climberLeft.clearFaults();
        m_climberLeft.setIdleMode(IdleMode.kBrake);
        m_climberRight.setInverted(RobotClimberConstants.kClimberRightInversion);
        m_climberLeft.setInverted(RobotClimberConstants.kClimberLeftInversion);

        m_rightPidController = m_climberRight.getPIDController();
        m_leftPidController = m_climberLeft.getPIDController();

        // encoders
        rightClimberEncoder = m_climberRight.getEncoder();
        leftClimberEncoder = m_climberLeft.getEncoder();

        // set PID coefficients
        m_rightPidController.setP(RobotClimberConstants.kP);
        m_leftPidController.setP(RobotClimberConstants.kP);
        m_rightPidController.setI(RobotClimberConstants.kI);
        m_leftPidController.setI(RobotClimberConstants.kI);
        m_rightPidController.setD(RobotClimberConstants.kD);
        m_leftPidController.setD(RobotClimberConstants.kD);
        m_rightPidController.setIZone(RobotClimberConstants.kIz);
        m_leftPidController.setIZone(RobotClimberConstants.kIz);
        m_rightPidController.setFF(RobotClimberConstants.kFF);
        m_leftPidController.setFF(RobotClimberConstants.kFF);
        m_rightPidController.setOutputRange(RobotClimberConstants.kMinOutput, RobotClimberConstants.kMaxOutput);
        m_leftPidController.setOutputRange(RobotClimberConstants.kMinOutput, RobotClimberConstants.kMaxOutput);
        SmartDashboard.putNumber("P Gain", pidP);

        // linear Motors
        leftHomer = new LinearActuatorHomer(0, m_climberLeft, -260);
        rightHomer = new LinearActuatorHomer(1, m_climberRight, -260);
    }

    public void leftClimb(double power) { // Negative is arm extention, positive is arm retraction
        if (leftHomer.limitTripped() && power > 0) { // if the limit switch is tripped
            m_climberLeft.set(0);
        } else {
            m_climberLeft.set(power);
        }
        SmartDashboard.putNumber("Left CLiber Power", power);
        SmartDashboard.putBoolean("Left Homer", leftHomer.limitTripped());

    }

    public void rightClimb(double power) { // Negative is arm extention, positive is arm retraction
        if (rightHomer.limitTripped() && power > 0) { // if the limit switch is tripped
            m_climberRight.set(0);
        } else {
            m_climberRight.set(power);
        }
        SmartDashboard.putNumber("Right CLiber Power", power);
        SmartDashboard.putBoolean("Right Homer", rightHomer.limitTripped());
    }

    public void autoClimb(double power) {

        double initalBearing = navxGyro.getRoll();
        double output = thetaController.calculate(initalBearing, 0);

        rightClimb(power + output / 2);
        leftClimb(power - output / 2);

    }
    
    public void periodic(){
        /*
        double p = SmartDashboard.getNumber("P", pidP);
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((pidP != RobotClimberConstants.kP)) { 
            m_rightPidController.setP(p); 
            pidP = p; 
        }
        m_rightPidController.setOutputRange(min, max);
        m_leftPidController.setOutputRange(min, max);
        */

        leftHomer.periodic();
        SmartDashboard.putNumber("Left Encoder", leftClimberEncoder.getPosition());
        rightHomer.periodic();
        SmartDashboard.putNumber("Right Encoder", rightClimberEncoder.getPosition());
    }
}
