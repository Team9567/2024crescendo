package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants.RobotChassisConstants;

public class RobotChassis extends SubsystemBase {

    public CANSparkMax leftCanSparkMax = new CANSparkMax(RobotChassisConstants.kLeftCanId, MotorType.kBrushless);
    public CANSparkMax rightCanSparkMax = new CANSparkMax(RobotChassisConstants.kRightCanId, MotorType.kBrushless);
    public CANSparkMax leftFollowerCanSparkMax = new CANSparkMax(RobotChassisConstants.kLeftFollowerCanId, MotorType.kBrushless);
    public CANSparkMax rightFollowerCanSparkMax = new CANSparkMax(RobotChassisConstants.kRightFollowerCanId, MotorType.kBrushless);
    public DifferentialDrive drivetrain = new DifferentialDrive(leftCanSparkMax, rightCanSparkMax);
    public double targetSpeed = 0;
    public double targetTurn = 0;
    public double currentSpeed = 0;
    public double currentTurn = 0;
    public RelativeEncoder leftEncoder = leftCanSparkMax.getEncoder();
    public RelativeEncoder rightEncoder = rightCanSparkMax.getEncoder();
    public AHRS navxGyro;
    public DifferentialDrivePoseEstimator poseEstimator;
    

    public RobotChassis(AHRS navxGyro, DifferentialDrivePoseEstimator poseEstimator) {

        this.navxGyro = navxGyro;
        this.poseEstimator = poseEstimator;

        for (CANSparkMax m : new CANSparkMax[] { leftCanSparkMax, rightCanSparkMax, leftFollowerCanSparkMax,
                rightFollowerCanSparkMax }) {
            m.clearFaults();
            m.setIdleMode(RobotChassisConstants.kMotorBrakeMode);
            m.setSmartCurrentLimit(RobotChassisConstants.kCurrentLimit, RobotChassisConstants.kCurrentLimit);
        }

        leftCanSparkMax.setInverted(false);
        rightCanSparkMax.setInverted(true);
        leftFollowerCanSparkMax.setInverted(true);
        rightFollowerCanSparkMax.setInverted(true);
        // configure followers
        leftFollowerCanSparkMax.follow(leftCanSparkMax);
        rightFollowerCanSparkMax.follow(rightCanSparkMax);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        leftEncoder.setPositionConversionFactor(RobotChassisConstants.kWheelCircumfrance/RobotChassisConstants.kMotorReduction);
        rightEncoder.setPositionConversionFactor(RobotChassisConstants.kWheelCircumfrance/RobotChassisConstants.kMotorReduction);



    }

    public void arcadeDrive(double power, double turn) {
        targetSpeed = power;
        targetTurn = turn * -1;
        updateSpeed();
    }

    public void periodic() {
        updateSpeed();
    }

    public void updateSpeed() {
        var speedDiffrence = targetSpeed - currentSpeed;
        var turnDiffrence = targetTurn - currentTurn;
        if (speedDiffrence > .5) {
            currentSpeed += .15;
        } else if (speedDiffrence > .15) {
            currentSpeed += .1;
        } else {
            currentSpeed = targetSpeed;
        }

        if (turnDiffrence > 0.5) {
            currentTurn += .15;
        } else if (speedDiffrence > .15) {
            currentTurn += .1;
        } else {
            currentTurn = targetTurn;
        }
        drivetrain.arcadeDrive(currentSpeed, currentTurn);
    }

}
