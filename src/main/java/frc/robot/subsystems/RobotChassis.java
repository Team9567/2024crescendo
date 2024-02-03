package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants.RobotChassisConstants;

public class RobotChassis extends SubsystemBase {

    public CANSparkMax leftCanSparkMax = new CANSparkMax(RobotChassisConstants.kLeftCanId, MotorType.kBrushless);
    public CANSparkMax rightCanSparkMax = new CANSparkMax(RobotChassisConstants.kRightCanId, MotorType.kBrushless);
    public CANSparkMax leftFollowerCanSparkMax = new CANSparkMax(RobotChassisConstants.kLeftFollowerCanId,
            MotorType.kBrushless);
    public CANSparkMax rightFollowerCanSparkMax = new CANSparkMax(RobotChassisConstants.kRightFollowerCanId,
            MotorType.kBrushless);
    public DifferentialDrive drivetrain = new DifferentialDrive(leftCanSparkMax, rightCanSparkMax);
    public double targetSpeed = 0;
    public double targetTurn = 0;
    public double currentSpeed = 0;
    public double currentTurn = 0;
    public boolean lowGear = false;
    public RelativeEncoder leftEncoder = leftCanSparkMax.getEncoder();
    public RelativeEncoder rightEncoder = rightCanSparkMax.getEncoder();
    public AHRS navxGyro;
    public DifferentialDrivePoseEstimator poseEstimator;
    public PIDController thetaController = new PIDController(1/180, 0, 0);
    public Field2d field;
    

    public RobotChassis(AHRS navxGyro, DifferentialDrivePoseEstimator poseEstimator, Field2d field) {

        this.navxGyro = navxGyro;
        this.poseEstimator = poseEstimator;
        this.field = field;

        for (CANSparkMax m : new CANSparkMax[] { leftCanSparkMax, rightCanSparkMax, leftFollowerCanSparkMax,
                rightFollowerCanSparkMax }) {
            m.clearFaults();
            m.setIdleMode(RobotChassisConstants.kMotorBrakeMode);
            m.setSmartCurrentLimit(RobotChassisConstants.kCurrentLimit, RobotChassisConstants.kCurrentLimit);
            m.setOpenLoopRampRate(RobotChassisConstants.rampRate);
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

        thetaController.enableContinuousInput(-180, 180);

    }

    public void setLowGear() {
        lowGear = true;
    }

    public void setHighGear() {
        lowGear = false;
    }

    public void arcadeDrive(double power, double turn) {
        if (lowGear == true) {
            targetSpeed = power / 2.5;
            targetTurn = turn / -2.5;
        } else {
            targetSpeed = power;
            targetTurn = turn * -1;
        }
        drivetrain.arcadeDrive(targetSpeed, targetTurn);
        SmartDashboard.putNumber("Target speed", targetSpeed);
        SmartDashboard.putNumber("Target turn", targetTurn);
    }

    public void periodic() {
        poseEstimator.update(navxGyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
        field.setRobotPose(poseEstimator.getEstimatedPosition());

        SmartDashboard.putData("field", field);
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
        /* 
        if(lowGear = true){
            drivetrain.arcadeDrive(currentSpeed/4, currentTurn/4);
        } else {
            drivetrain.arcadeDrive(currentSpeed, currentTurn);
        }
        */
    }

    public void chassisToBearing(double targetRotation){


        double initalBearing = navxGyro.getRotation2d().getDegrees();
        double output = thetaController.calculate(initalBearing, targetRotation);

        arcadeDrive(0, output); // if turns as fast as possible invert output

    }



}
