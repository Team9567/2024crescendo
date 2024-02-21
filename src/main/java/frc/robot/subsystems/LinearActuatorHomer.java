package frc.robot.subsystems;

import java.nio.channels.Channel;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.units.Unit;
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

    public void periodic() {

        // Runs the motors on half speed, unless the limit swith is pressed.
        if (homerChannel.get()) {
            if (homingMotor.get() < 0) {
                homingMotor.set(0);
                homingMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
                homingMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, m_limit);
                homingMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
                homingMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, 3);
            }
            if (homingMotor.getEncoder().getVelocity() == 0) { // if stopped
                    homingMotor.getEncoder().setPosition(0); // Home the motor to position = 0
            }
            
            // get encoder home position
            /*
             * while (encoderHomePosition.getVelocity() != 0) {
             * }
             * ;
             */// Wait Until stopped
        }

    }
    /*
     * SparkLimitSwitch homeSwitch;
     * boolean homed = false;
     * float limit = 0;
     * double rate = 0.0;
     * 
     * public LinearActuatorHomer(float limit, double rate, SparkLimitSwitch
     * homeSwitch) {
     * this.limit = limit;
     * this.rate = rate;
     * this.homeSwitch = homeSwitch;
     * //enables the limit switch. Stops the motor once the switch is triggered
     * homeSwitch.enableLimitSwitch(true);
     * 
     * }
     * 
     * 
     * public void homeStep(CANSparkMax homingMotor) {
     * if (homed == true) {
     * return;
     * }
     * 
     * 
     * if (homeSwitch.isPressed() == true) {
     * // homingMotor.set(0);
     * //runs and stops the motor when we are homed, our keeps going otherwise
     * homingMotor.getEncoder().setPosition(0);
     * homingMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
     * homingMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, limit);
     * homed = true;
     * return;
     * } else {
     * homingMotor.set(rate);
     * }
     * 
     * }
     * 
     * /*
     * public void softStopTest(CANSparkMax homingMotor) {
     * if (homed == false){
     * //If the motor is running the opposite direction of home stop.
     * if(homingMotor.get()/rate < 0){
     * homingMotor.set(0);
     * }
     * return;
     * }
     * //checks the direction of the motors and stops it if its going the wrong way
     * if (homingMotor.get() < 0 && homingMotor.getEncoder().getPosition() < 3) {
     * homingMotor.set(0);
     * }
     * //checks the direction of the motors and stops it if its going the wrong way
     * if (homingMotor.get() > 0 && homingMotor.getEncoder().getPosition() > (limit
     * - 3)) {
     * homingMotor.set(0);
     * }
     * }
     */
}
