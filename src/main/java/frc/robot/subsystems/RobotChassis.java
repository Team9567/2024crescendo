package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.revrobotics.CANSparkMax.IdleMode;

public class RobotChassis extends SubsystemBase {
    public CANSparkMax leftCanSparkMax = new CANSparkMax(1, MotorType.kBrushless);
    public CANSparkMax rightCanSparkMax = new CANSparkMax(3, MotorType.kBrushless);
    public CANSparkMax leftFollowerCanSparkMax = new CANSparkMax(2, MotorType.kBrushless);
    public CANSparkMax rightFollowerCanSparkMax = new CANSparkMax(4, MotorType.kBrushless);
    public DifferentialDrive drivetrain = new DifferentialDrive(leftCanSparkMax, rightCanSparkMax);
    public double targetSpeed = 0;
    public double targetTurn = 0;
    public double currentSpeed = 0;
    public double currentTurn = 0;

    public RobotChassis(){

        for (CANSparkMax m : new CANSparkMax[] { leftCanSparkMax, rightCanSparkMax, leftFollowerCanSparkMax,rightFollowerCanSparkMax }) {
            m.clearFaults();
            m.setIdleMode(IdleMode.kBrake);
            m.setSmartCurrentLimit(240 / 4, 240 / 4);
        }
        leftCanSparkMax.setInverted(false);
        rightCanSparkMax.setInverted(true);

        // configure followers
        leftFollowerCanSparkMax.follow(leftCanSparkMax);
        rightFollowerCanSparkMax.follow(rightCanSparkMax);
    }

    public void arcadeDrive(double power, double turn) {
        targetSpeed = power;
        targetTurn = turn * -1;
        updateSpeed();
    }
    public void periodic(){
        updateSpeed();
    }
    public void updateSpeed(){
        var speedDiffrence = targetSpeed - currentSpeed;
        var turnDiffrence = targetTurn - currentTurn;
        if (speedDiffrence > .5){
            currentSpeed += .15;
        }else if (speedDiffrence > .15){
            currentSpeed += .1;
        }else{
            currentSpeed = targetSpeed;
        }

        if (turnDiffrence > 0.5){
            currentTurn += .15;
        }else if (speedDiffrence > .15){
            currentTurn += .1;
        }else{
            currentTurn = targetTurn;
        }
        drivetrain.arcadeDrive(currentSpeed, currentTurn);
    }

}

