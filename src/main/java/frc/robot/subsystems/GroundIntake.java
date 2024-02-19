package frc.robot.subsystems;

//imports
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.lang.annotation.Target;

import org.ejml.equation.Function;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

import au.grapplerobotics.LaserCan; //Distance sensor, grappleHook, time of flight(TOF)
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Constants.GroundIntakeConstants.GroundIntakeStateMachine;
import frc.robot.Constants.RobotChassisConstants;
import frc.robot.subsystems.LinearActuatorHomer;

//Subystem class
public class GroundIntake extends SubsystemBase {

        // Variables
        // Current groundIntakePivotAngle
        // Curent groundIntakeHeight
        // Current groundIntakeState
        // Current groundIntakeSpeed, MotorOutake Speed, Dejamming/shooting
        // 2 Home switches
        // When note is in -- Boolean

        // Constants
        // Enum Height refering to state -- Constant for Home
        // Enum Pivot angle refering to State -- Angles for StorageLow, StorageHigh,
        // Ground, Unknown, Inside perimeter storage low -- DECLRES THE ANGLES FOR THE
        // STATE
        // Enum intakeSpeed
        // Enum state machine -- DECLARES THE STATES
        // Distance when note is in

        // Motors
        // M8 and M9, linear acuators for height.
        // M10, Motor for pivot
        // M11 and M12, Intaking notes.
        // Make one of the arm motors follow the other arm

        public GroundIntakeStateMachine currentIntakeState = GroundIntakeStateMachine.S0_Unknown;
        public GroundIntakeStateMachine targetState = GroundIntakeStateMachine.S2_PositionLow;

        // public GroundIntakeStateMachine.enum unknownState =
        // GroundIntakeStateMachine.S0_Unknown;

        // Linear Motors -- Height
        public CANSparkMax heightPositionLeftMotor = new CANSparkMax(GroundIntakeConstants.kHeightPositionLeftDeviceID,
                        MotorType.kBrushless);
        public CANSparkMax heightPositionRightMotor = new CANSparkMax(
                        GroundIntakeConstants.kHeightPositionRightDeviceID,
                        MotorType.kBrushless);

        // Rotational Motors -- Pivot
        public CANSparkMax pivotPositionMotor = new CANSparkMax(GroundIntakeConstants.kPivotPositionMotorDeviceID,
                        MotorType.kBrushless); // TODO Motor typeCheck later

        // RPM motors -- Intake
        public CANSparkMax leftIntakeRPMMotor = new CANSparkMax(GroundIntakeConstants.kLeftIntakeRPMMotorDeviceID,
                        MotorType.kBrushless); // TODO Motor typeCheck later
        public CANSparkMax rightIntakeRPMMotorFollower = new CANSparkMax(
                        GroundIntakeConstants.kRightIntakeRPMMotor_Follower,
                        MotorType.kBrushless); // TODO Motor typeCheck later

        // Encoders -- For the height
        public RelativeEncoder heightPositionLeftEncoder = heightPositionLeftMotor.getEncoder();
        public RelativeEncoder heightPositionRightEncoder = rightIntakeRPMMotorFollower.getEncoder();

        // Encoders -- For Pivot
        public RelativeEncoder pivotAngleEncoder = pivotPositionMotor.getEncoder();

        // Encoders -- For RPM motors
        public RelativeEncoder leftGroundIntakeEncoder = leftIntakeRPMMotor.getEncoder();
        public RelativeEncoder rightGroundIntakeEncoder = rightIntakeRPMMotorFollower.getEncoder();

        // Declare NoteSensor
        LaserCan noteSensor = new LaserCan(GroundIntakeConstants.kLaserCanId);

        // Make the homing switches
        /*
        LinearActuatorHomer leftElevatorHomer = new LinearActuatorHomer(GroundIntakeConstants.kLeftElevatorLimit,
                        GroundIntakeConstants.kLeftElevatorHomingSpeed,
                        heightPositionLeftMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed));
        LinearActuatorHomer rightElevatorHomer = new LinearActuatorHomer(GroundIntakeConstants.kRightElevatorLimit,
                        GroundIntakeConstants.kRightElevatorHomingSpeed,
                        heightPositionRightMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed));
         */
        public GroundIntake() {
                // Code for getting a measurement from a laser can
                // LaserCan.Measurement measurement = noteSensor.getMeasurement();

                for (CANSparkMax m : new CANSparkMax[] { heightPositionLeftMotor, heightPositionRightMotor,
                                pivotPositionMotor,
                                leftIntakeRPMMotor, rightIntakeRPMMotorFollower }) {
                        m.clearFaults();
                        m.setIdleMode(RobotChassisConstants.kMotorBrakeMode);
                        m.setSmartCurrentLimit(RobotChassisConstants.kCurrentLimit,
                                        RobotChassisConstants.kCurrentLimit);
                        m.setOpenLoopRampRate(RobotChassisConstants.rampRate);
                }
                // Declare encoders
                // Height left and right
                heightPositionLeftEncoder.setPosition(0);
                heightPositionRightEncoder.setPosition(0);

                // PIVOT ANGLE, Absolute incoder
                pivotAngleEncoder.setPosition(0);

                // Ground intake left and right
                leftGroundIntakeEncoder.setPosition(0);
                rightGroundIntakeEncoder.setPosition(0);

                // Declaring the right ground intake motor a follower
                rightIntakeRPMMotorFollower.follow(leftIntakeRPMMotor);
        }
        /*
        public void periodic() {
                // rightElevatorHomer.softStopTest(heightPositionRightMotor);
                // leftElevatorHomer.softStopTest(heightPositionLeftMotor);
                rightElevatorHomer.homeStep(heightPositionRightMotor);
                leftElevatorHomer.homeStep(heightPositionLeftMotor);
                transitionState();
        }
         */
        public void setTarget(GroundIntakeStateMachine target) {
                targetState = target;
        }

        public void transitionState() {
                SmartDashboard.putString("Intake/Target", targetState.name());
                SmartDashboard.putString("Intake/Current", currentIntakeState.name());
                if (currentIntakeState == targetState) {
                        return;
                }

                var nextState = currentIntakeState.nextStepTo(targetState);
                SmartDashboard.putString("Intake/Next", nextState.name());
                // Takes the current state and advances next state forward
                if (currentIntakeState == GroundIntakeStateMachine.S0_Unknown
                                && nextState == GroundIntakeStateMachine.S2_PositionLow) {
                        S0ToS2();

                } else if (currentIntakeState == GroundIntakeStateMachine.S1_GroundIntake
                                && nextState == GroundIntakeStateMachine.S2_PositionLow) {
                        S1ToS2();

                } else if (currentIntakeState == GroundIntakeStateMachine.S2_PositionLow
                                && nextState == GroundIntakeStateMachine.S3_PositionHigh) {
                        S2ToS3();

                } else if (currentIntakeState == GroundIntakeStateMachine.S3_PositionHigh
                                && nextState == GroundIntakeStateMachine.S4_PositionDunk) {
                        S3ToS4();

                        // Takes the current state and advances next state backward
                } else if (currentIntakeState == GroundIntakeStateMachine.S2_PositionLow
                                && nextState == GroundIntakeStateMachine.S1_GroundIntake) {
                        S2ToS1();

                } else if (currentIntakeState == GroundIntakeStateMachine.S3_PositionHigh
                                && nextState == GroundIntakeStateMachine.S2_PositionLow) {
                        S3ToS2();

                } else if (currentIntakeState == GroundIntakeStateMachine.S4_PositionDunk
                                && nextState == GroundIntakeStateMachine.S3_PositionHigh) {
                        S4ToS3();
                }
        }

        public void S0ToS2() {
                SmartDashboard.putString("Intake/Action", "S0ToS2");
                currentIntakeState = GroundIntakeStateMachine.S2_PositionLow;
        }

        public void S1ToS2() {
                SmartDashboard.putString("Intake/Action", "S1ToS2");
                currentIntakeState = GroundIntakeStateMachine.S2_PositionLow;
        }

        public void S2ToS3() {
                SmartDashboard.putString("Intake/Action", "S2ToS3");
                currentIntakeState = GroundIntakeStateMachine.S3_PositionHigh;
        }

        public void S3ToS4() {
                SmartDashboard.putString("Intake/Action", "S3ToS4");
                currentIntakeState = GroundIntakeStateMachine.S4_PositionDunk;
        }

        public void S2ToS1() {
                SmartDashboard.putString("Intake/Action", "S2ToS1");
                currentIntakeState = GroundIntakeStateMachine.S1_GroundIntake;
        }

        public void S3ToS2() {
                SmartDashboard.putString("Intake/Action", "S3ToS2");
                currentIntakeState = GroundIntakeStateMachine.S2_PositionLow;
        }

        public void S4ToS3() {
                SmartDashboard.putString("Intake/Action", "S4ToS3");
                currentIntakeState = GroundIntakeStateMachine.S3_PositionHigh;
        }

        // need a helper function that takes a "now" state and a "target" state, and
        // returns the next step state

}