package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LinearActuatorHomer {
    DigitalInput homerChannel;
    float m_limit;
    CANSparkMax homingMotor;
    boolean isHomed = false ;

    public LinearActuatorHomer(int DIOPort, CANSparkMax homerMotor, float limit) {

        homingMotor = homerMotor;
        m_limit = limit;
        homerChannel = new DigitalInput(DIOPort);
        homerMotor.set(.15);
    }

    public boolean limitTripped(){
        return !homerChannel.get();
    }

    public boolean isHomed(){
        return isHomed;
    }    

    public void periodic() {

        //set the soft limits
        if (limitTripped()) { //looks to see if the limit switch is tripped
            if (homingMotor.get() > 0) { //if the acuator is heading towards the limit switch 
                homingMotor.set(0); 
                homingMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true); //upper -304 limmit left
                homingMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, m_limit); // 1upp -380 limmit right
                homingMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
                homingMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, -10); //Three encoder ticks above the limit switch
                homingMotor.getEncoder().setPosition(0); // Home the motor to position = 0
                isHomed = true;
            }
        }
    }
}
