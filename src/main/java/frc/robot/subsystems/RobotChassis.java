package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkFlex;
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

    public CANSparkFlex leftCanSparkFlex = new CANSparkFlex(RobotChassisConstants.kLeftCanId, MotorType.kBrushless);
    public CANSparkFlex rightCanSparkFlex = new CANSparkFlex(RobotChassisConstants.kRightCanId, MotorType.kBrushless);
    public CANSparkFlex leftFollowerCanSparkFlex = new CANSparkFlex(RobotChassisConstants.kLeftFollowerCanId,
            MotorType.kBrushless);
    public CANSparkFlex rightFollowerCanSparkFlex = new CANSparkFlex(RobotChassisConstants.kRightFollowerCanId,
            MotorType.kBrushless);
    public DifferentialDrive drivetrain = new DifferentialDrive(leftCanSparkFlex, rightCanSparkFlex);
    public boolean lowGear = false;
    public RelativeEncoder leftEncoder = leftCanSparkFlex.getEncoder();
    public RelativeEncoder rightEncoder = rightCanSparkFlex.getEncoder();
    public AHRS navxGyro;
    public DifferentialDrivePoseEstimator poseEstimator;
    public PIDController thetaController = new PIDController(1.0/90.0, 0, 0);
    public Field2d field;


    public RobotChassis(AHRS navxGyro, DifferentialDrivePoseEstimator poseEstimator, Field2d field) {

        this.navxGyro = navxGyro;
        this.poseEstimator = poseEstimator;
        this.field = field;

        for (CANSparkFlex m : new CANSparkFlex[] { leftCanSparkFlex, rightCanSparkFlex, leftFollowerCanSparkFlex,
                rightFollowerCanSparkFlex }) {
            m.clearFaults();
            m.setIdleMode(RobotChassisConstants.kMotorBrakeMode);
            m.setSmartCurrentLimit(RobotChassisConstants.kCurrentLimit, RobotChassisConstants.kCurrentLimit);
            m.setOpenLoopRampRate(RobotChassisConstants.rampRate);
        }

        leftCanSparkFlex.setInverted(false);
        rightCanSparkFlex.setInverted(true);
        leftFollowerCanSparkFlex.setInverted(true);
        rightFollowerCanSparkFlex.setInverted(true);
        // configure followers
        leftFollowerCanSparkFlex.follow(leftCanSparkFlex);
        rightFollowerCanSparkFlex.follow(rightCanSparkFlex);
        //Lower can reporting rate for motor telemetry - 20ms -> 500ms
        leftCanSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500); 
        leftCanSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        rightCanSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        rightCanSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        //Lower can reporting rate for follower motor telemtry - 10ms -> 100ms / 20ms -> 500ms
        leftFollowerCanSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        leftFollowerCanSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        leftFollowerCanSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        rightFollowerCanSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        rightFollowerCanSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        rightFollowerCanSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        leftEncoder.setPositionConversionFactor(
                RobotChassisConstants.kWheelCircumfrance / RobotChassisConstants.kMotorReduction);
        rightEncoder.setPositionConversionFactor(
                RobotChassisConstants.kWheelCircumfrance / RobotChassisConstants.kMotorReduction);

        thetaController.enableContinuousInput(-180, 180);

    }

    public void setLowGear() {
        lowGear = true;
    }

    public void setHighGear() {
        lowGear = false;
    }

    public void arcadeDrive(double power, double turn) {
        double targetSpeed = 0.0;
        double targetTurn = 0.0;
        if (lowGear == true) {
            targetSpeed = power / RobotChassisConstants.kLowGearSpeedDivider;
            targetTurn = -1 * turn / RobotChassisConstants.kLowGearSpeedDivider;
        } else {
            targetSpeed = power;
            targetTurn = -1 * turn;
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

    public void chassisToBearing(double targetRotation) {

        double initalBearing = navxGyro.getRotation2d().getDegrees();
        double output = thetaController.calculate(initalBearing, targetRotation);
        SmartDashboard.putNumber("Target", targetRotation);
        SmartDashboard.putNumber("Initial", initalBearing);
        SmartDashboard.putNumber("output", output); 

        drivetrain.arcadeDrive(0, output * 0.5); // if turns as fast as possible invert output

    }

}
