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
    //orient button 

    //Keybindings
    public static final int kOperatorButtonLaunch = 2; //Button B
    public static final int kOperatorButtonIntake = 3; //Button X

    public static final int kOperatorAxisLeftClimb = 1; //Left Analog Vertical axis
    public static final int kOperatorAxisRightClimb = 5; //Right Analog Vertical axis

    public static final int kDriverButtonGear = 3; //button x driver controller

    public static final double klauncherRunTimeConstant = 3.0; //TODO change later
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
    public static final double kLauncherSpeed = 1;
    public static final double kLaunchFeederSpeed = 1;
    public static final double kIntakeLauncherSpeed = -1;
    public static final double kIntakeFeederSpeed = -.2;

    public static final double kLauncherDelay = 1;

    public static final IdleMode kLaunchBrakeMode = IdleMode.kBrake;
    public static final IdleMode kFeedBrakeMode = IdleMode.kBrake;
  }

  public static class RobotConstants {
    public static final int rightClimberDIO = 0;
    public static final int leftClimberDIO = 1;
    public static final int groundIntakeRightClimberDIO = 2;
    public static final int groundIntakeLeftClimberDIO = 3;
  }

  public static class autonomousCommand {
    public static final double kPosATurn1 = 0;
    public static final double kPosCTurn1 = 0;
    public static final double kPosBTurn1 = 2;

    public static final double kPosABack1 = 2;
    public static final double kPosCBack1 = 0.5;
    public static final double kPosBBack1 = 5;

    public static final double kPosATurn2 = 1.65;
    public static final double kPosCTurn2 = 0.26;
    public static final double kPosBTurn2 = 0.24;

  
    public static final double kPosABack2 = 2.8;
    public static final double kPosCBack2 = 3;
    public static final double kPosBBack2 = 2.8;

  }

  public static class RobotChassisConstants {
    public static final int kCurrentLimit = 60;  // rev robotics recommendations 40-60
    public static final int kLeftCanId = 1;
    public static final int kRightCanId = 3;
    public static final int kLeftFollowerCanId = 2;
    public static final int kRightFollowerCanId = 4;
    public static final IdleMode kMotorBrakeMode = IdleMode.kBrake;
    public static final double kTrackWidth = 1.0; //TODO change later

    //The Ramp Rate
    public static final double rampRate = .25;
    public static final double kLowGearSpeedDivider = 2.5;

    //public static final double kMotorReduction = 1.0/8.46;
    public static final double kMotorReduction = 8.46/1.0;
    public static final double kWheelDiameter = Units.inchesToMeters(6.0); //TODO change later
    public static final double kWheelCircumfrance = kWheelDiameter * Math.PI;
  }

  public static class RobotClimberConstants {
    public static final int kClimberRightID = 13;
    public static final int kClimberLeftID = 12;

    public static final boolean kClimberRightInversion = false;
    public static final boolean kClimberLeftInversion = false;

    // PID coefficients
    public static final double kFF = 0.000015;//TODO Tune 
    public static final double kP = 6e-5;//TODO TUNE 
    public static final double kI = 0;
    public static final double kD = 0; 
    public static final double kIz = 0; 
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;
    public static final double maxRPM = 5700;//TODO turn down
  }

  public static final class UnderTheBumperConstants {
    public static final int kUnderTheBumperCanId = 1; //TODO replace this later
    public static final int kUnderTheBumperFollowerID = 1; //TODO replace this later
    public static final double kUnderTheBumperMotorSpeed = .5; //TODO update later
    public static final int kRingDistanceMM = 10; //TODO change later
  }
}
