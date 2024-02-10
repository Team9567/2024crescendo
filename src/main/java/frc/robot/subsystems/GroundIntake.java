package frc.robot.subsystems;
//imports
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
//Subystem class
public class GroundIntake extends SubsystemBase {
    
    //Variables
    //Current groundIntakePivotAngle
    //Curent groundIntakeHeight
    //Current groundIntakeState
    //Current groundIntakeSpeed, MotorOutake Speed, Dejamming/shooting
    //2 Home switches
    //When note is in -- Boolean
    

    //Constants
    //Enum Height refering to state -- Constant for Home
    //Enum Pivot angle refering to State -- Angles for StorageLow, StorageHigh, Ground, Unknown, Inside perimeter storage low -- DECLRES THE ANGLES FOR THE STATE
    //Bool, when note is in
    //intakeSpeed variable
    //Angles for the specific states
    //Enum state machine -- DECLARES THE STATES
    //Distance when note is in

    public CANSparkMax leftCanSparkMax = new CANSparkMax(RobotChassisConstants.kLeftCanId, MotorType.kBrushless);
    public CANSparkMax rightCanSparkMax = new CANSparkMax(RobotChassisConstants.kRightCanId, MotorType.kBrushless);
    public CANSparkMax leftFollowerCanSparkMax = new CANSparkMax(RobotChassisConstants.kLeftFollowerCanId,
            MotorType.kBrushless);
    public CANSparkMax rightFollowerCanSparkMax = new CANSparkMax(RobotChassisConstants.kRightFollowerCanId,
            MotorType.kBrushless);
    public DifferentialDrive drivetrain = new DifferentialDrive(leftCanSparkMax, rightCanSparkMax);
    public boolean lowGear = false;
    public RelativeEncoder leftEncoder = leftCanSparkMax.getEncoder();
    public RelativeEncoder rightEncoder = rightCanSparkMax.getEncoder();
    public AHRS navxGyro;
    public DifferentialDrivePoseEstimator poseEstimator;
    public PIDController thetaController = new PIDController(1 / 180, 0, 0);
    public Field2d field;

    public GroundIntake(AHRS navxGyro, DifferentialDrivePoseEstimator poseEstimator, Field2d field) {
        
        //Declare NoteSensor

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
        
        //Motors
        //M8 and M9, linear acuators for height.
        //M10, Motor for pivot
        //M11 and M12, Intaking notes.
        leftCanSparkMax.setInverted(false);
        rightCanSparkMax.setInverted(true);
        leftFollowerCanSparkMax.setInverted(true);
        rightFollowerCanSparkMax.setInverted(true);

        //Make one of the arm motors follow the other arm
        leftFollowerCanSparkMax.follow(leftCanSparkMax);
        rightFollowerCanSparkMax.follow(rightCanSparkMax);

        //Declare encoders
        //PIVOT ANGLE

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        //heightPositionLeft.setPosition(0);
        //heightPositionRight.setPosition(0);
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

        arcadeDrive(0, output); // if turns as fast as possible invert output

    }

}
