package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotChassis extends SubsystemBase {
    public CANSparkMax leftCanSparkMax = new CANSparkMax(1, MotorType.kBrushless);
    public CANSparkMax rightCanSparkMax = new CANSparkMax(3, MotorType.kBrushless);
    public CANSparkMax leftFollowerCanSparkMax = new CANSparkMax(2, MotorType.kBrushless);
    public CANSparkMax rightFollowerCanSparkMax = new CANSparkMax(4, MotorType.kBrushless);
    public DifferentialDrive drivetrain = new DifferentialDrive(leftCanSparkMax, rightCanSparkMax);

    public RobotChassis() {

        for (CANSparkMax m : new CANSparkMax[] { leftCanSparkMax, rightCanSparkMax, leftFollowerCanSparkMax,
                rightFollowerCanSparkMax }) {
            m.setIdleMode(IdleMode.kBrake);
            m.setSmartCurrentLimit(240 / 4, 240 / 4);
            m.clearFaults();
        }
        leftCanSparkMax.setInverted(false);
        rightCanSparkMax.setInverted(true);

        // configure followers
        leftFollowerCanSparkMax.follow(leftCanSparkMax);
        rightFollowerCanSparkMax.follow(rightCanSparkMax);
    }

    public void arcadeDrive(double power, double turn) {
        drivetrain.arcadeDrive(power, turn);
        
    }

}
