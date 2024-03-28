package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants.RobotChassisConstants;

public class RobotChassis extends SubsystemBase {

    public CANSparkFlex leftCanSparkMax = new CANSparkFlex(RobotChassisConstants.kLeftCanId, MotorType.kBrushless);
    public CANSparkFlex rightCanSparkMax = new CANSparkFlex(RobotChassisConstants.kRightCanId, MotorType.kBrushless);
    public CANSparkFlex leftFollowerCanSparkMax = new CANSparkFlex(RobotChassisConstants.kLeftFollowerCanId,
            MotorType.kBrushless);
    public CANSparkFlex rightFollowerCanSparkMax = new CANSparkFlex(RobotChassisConstants.kRightFollowerCanId,
            MotorType.kBrushless);
    public DifferentialDrive drivetrain = new DifferentialDrive(leftCanSparkMax, rightCanSparkMax);
    public boolean lowGear = false;
    public RelativeEncoder leftEncoder = leftCanSparkMax.getEncoder();
    public RelativeEncoder rightEncoder = rightCanSparkMax.getEncoder();
    public AHRS navxGyro;
    public DifferentialDrivePoseEstimator poseEstimator;
    public PIDController thetaController = new PIDController(1.0/90.0, 0, 0);
    public Field2d field;


    public RobotChassis(AHRS navxGyro, DifferentialDrivePoseEstimator poseEstimator, Field2d field) {

        this.navxGyro = navxGyro;
        this.poseEstimator = poseEstimator;
        this.field = field;

        for (CANSparkFlex m : new CANSparkFlex[] { leftCanSparkMax, rightCanSparkMax, leftFollowerCanSparkMax,
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
        power /=2; 
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
        //poseEstimator.update(navxGyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
        //field.setRobotPose(poseEstimator.getEstimatedPosition());

        //SmartDashboard.putData("field", field);
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
