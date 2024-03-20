package frc.robot;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    // Port numbers for driver and operator gamepads. These correspond with the
    // numbers on the USB
    // tab of the DriverStation

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    // orient button

    // Keybindings
    public static final int kOperatorButtonLaunch = 2; // Button B
    public static final int kOperatorButtonIntake = 3; // Button X

    public static final int kOperatorButtonAmp = 4; // Button Y

    public static final int kOperatorAxisLeftClimb = 1; // Left Analog Vertical axis
    public static final int kOperatorAxisRightClimb = 5; // Right Analog Vertical axis

    public static final int kDriverButtonGear = 3; // button x driver controller
    public static final double klauncherRunTimeConstant = 3.0; //TODO change later
    public static final int kDriveOrientApriltag = 4; // ??????

  }

  public static class DrivetrainConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kLeftRearID = 2;
    public static final int kLeftFrontID = 1;
    public static final int kRightRearID = 4;
    public static final int kRightFrontID = 3;

    // Current limit for drivetrain motors
    public static final int kCurrentLimit = 60;
  }

  public static class LauncherConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kFeederID = 5;
    public static final int kLauncherID = 6;

    // Current limit for launcher and feed wheels
    public static final int kLauncherCurrentLimit = 60;
    public static final int kFeedCurrentLimit = 80;

    // Speeds for wheels when intaking and launching. Intake speeds are negative to
    // run the wheels
    // in reverse
    public static final double kLauncherForIntakeSpeed = .2;
    public static final double kLauncherSpeed = 1;
    public static final double kLaunchFeederSpeed = 1;
    public static final double kIntakeLauncherSpeed = -1;
    public static final double kIntakeFeederSpeed = -.2;
    public static final double kPositionSpeed = -.2;

    public static final double kAmpLaunchSpeed = 0.1;

    public static final double kLauncherDelay = 1;

    public static final IdleMode kLaunchBrakeMode = IdleMode.kBrake;
    public static final IdleMode kFeedBrakeMode = IdleMode.kBrake;

    //Run the motors for intaking with the ground intake
    public static final double kIntakeForShooterLauncherSpeed = .45;
  }

  public static class RobotConstants {
    public static final int rightClimberDIO = 0;
    public static final int leftClimberDIO = 1;
    public static final int groundIntakeRightClimberDIO = 2;
    public static final int groundIntakeLeftClimberDIO = 3;
  }

  public static class autonomousCommand {

    // blue STARTING
    /////// BLUE 15 SECOND AUTO VARIABLES /////
    /////// BLUE 15 SECOND AUTO VARIABLES /////
    public static final double kBlueShortBack1 = 0.5;
    public static final double kBlueLongBack1 = 3; // 0.5
    // public static final double kPosBBack1 = 5;

    public static final double kBlueShortTurn2 = 1.65;
    public static final double kBlueLongTurn2 = 0.875;// 0.26
    // public static final double kPosBTurn2 = 0.24;
    public static final double kBlueShortTurnDistance2 = 142.0; // front is back
    public static final double kBlueLongTurnDistance2 = -125.0; // front is back

    public static final double kBlueShortBack2 = 3;
    public static final double kBlueLongBack2 = 3; // 3
    // public static final double kPosBBack2 = 2.8;

    // RED STARTING
    /////// RED 15 SECOND AUTO VARIABLES /////
    /////// RED 15 SECOND AUTO VARIABLES /////
    public static final double kRedShortBack1 = 0.5;// 2
    public static final double kRedLongBack1 = 4;
    // public static final double kPosBBack1 = 5;

    public static final double kRedShortTurn2 = 1.65;
    public static final double kRedLongTurn2 = 1;
    // public static final double kPosBTurn2 = 0.24;
    public static final double kRedShortTurnDistance2 = 100.0; // front is back
    public static final double kRedLongTurnDistance2 = 135.0; // front is back

    public static final double kRedShortBack2 = 3; // 2
    public static final double kRedLongBack2 = 3;

    public static final int kClockWise = 1; // do we want a negation here? is this too much complexity and confusing????
                                            // NateO - 10:36AM 3/2/24
    public static final int KCounterClockWise = -1; // do we want a negation here? is this too much complexity and
                                                    // confusing???? NateO - 10:36AM 3/2/24

  }

  public static class RobotChassisConstants {
    public static final int kCurrentLimit = 60; // rev robotics recommendations 40-60
    public static final int kLeftCanId = 1;
    public static final int kRightCanId = 3;
    public static final int kLeftFollowerCanId = 2;
    public static final int kRightFollowerCanId = 4;
    public static final IdleMode kMotorBrakeMode = IdleMode.kBrake;
    public static final double kTrackWidth = 1.0; // TODO change later

    // The Ramp Rate
    public static final double rampRate = .25;
    public static final double kLowGearSpeedDivider = 2.5;

    // public static final double kMotorReduction = 1.0/8.46;
    public static final double kMotorReduction = 8.46 / 1.0;
    public static final double kWheelDiameter = Units.inchesToMeters(6.0); // TODO change later
    public static final double kWheelCircumfrance = kWheelDiameter * Math.PI;
  }

  public static class RobotClimberConstants {
    public static final int kClimberRightID = 13;
    public static final int kClimberLeftID = 12;

    public static final boolean kClimberRightInversion = false;
    public static final boolean kClimberLeftInversion = false;

    // PID coefficients
    public static final double kFF = 0.000015;// TODO Tune
    public static final double kP = 6e-5;// TODO TUNE
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    public static final double maxRPM = 5700;// TODO turn down
  }

  public static final class UnderTheBumperConstants {
    public static final int kUnderTheBumperCanId = 14; //Lower Track
    public static final int kUnderTheBumperFollowerID = 15; //Upper Track
    public static final double kUnderTheBumperMotorShootingSpeed = .1;
    public static final double kUnderTheBumperMotorIntakeSpeed = 1;
    public static final double kUnderTheBumperMotorExtakeSpeed = .15;
    public static final int kRingDistanceMM = 300; //TODO change later
  }
}

