package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

public class LinearActuatorHomer {
    DigitalInput homerChannel;
    float m_limit;
    CANSparkMax homingMotor;

    public LinearActuatorHomer(int DIOPort, CANSparkMax homerMotor, float limit) {

        homingMotor = homerMotor;
        m_limit = limit;
        homerChannel = new DigitalInput(DIOPort);
        homerMotor.set(-.1);
    }

    public boolean limitTripped(){
        return homerChannel.get();
    }

    public void periodic() {
        //set the soft limits
        if (homerChannel.get()) { //looks to see if the limit switch is tripped
            if (homingMotor.get() < 0) { //if the speed is less then 0 the acuator is heading towards the limit switch 
                homingMotor.set(0); 
                homingMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
                homingMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, m_limit);
                homingMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
                homingMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, 3); //Three encoder ticks above the limit switch
            }

            if (homingMotor.getEncoder().getVelocity() == 0) { // if stopped 
                    homingMotor.getEncoder().setPosition(0); // Home the motor to position = 0
            }  
        }
    }
}
