package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

public class LinearActuatorHomer {
    SparkLimitSwitch homeSwitch;
    boolean homed = false;
    float limit = 0;
    double rate = 0.0;

    public LinearActuatorHomer(float limit, double rate, SparkLimitSwitch homeSwitch) {
        this.limit = limit;
        this.rate = rate;
        this.homeSwitch = homeSwitch;
        //enables the limit switch. Stops the motor once the switch is triggered
        homeSwitch.enableLimitSwitch(true);

    }


    public void homeStep(CANSparkMax homingMotor) {
        if (homed == true) {
            return;
        }


        if (homeSwitch.isPressed() == true) {
            // homingMotor.set(0);
            //runs and stops the motor when we are homed, our keeps going otherwise
            homingMotor.getEncoder().setPosition(0);
            homingMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
            homingMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, limit);
            homed = true;
            return;
        } else {
            homingMotor.set(rate);
        }
        
    }
    
    /* 
    public void softStopTest(CANSparkMax homingMotor) {
        if (homed == false){
            //If the motor is running the opposite direction of home stop.
            if(homingMotor.get()/rate < 0){
                homingMotor.set(0);
            }
            return;
        }
        //checks the direction of the motors and stops it if its going the wrong way
        if (homingMotor.get() < 0 && homingMotor.getEncoder().getPosition() < 3) {
            homingMotor.set(0);
        }
        //checks the direction of the motors and stops it if its going the wrong way
        if (homingMotor.get() > 0 && homingMotor.getEncoder().getPosition() > (limit - 3)) {
            homingMotor.set(0);
        }
    }
    */
}
