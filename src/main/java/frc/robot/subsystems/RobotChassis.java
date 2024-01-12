package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
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
    //public DifferentialDrive drivetrain = new DifferentialDrive(null, null);

    public void arcadeDrive(double d, double rawAxis) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'arcadeDrive'");
    }
public RobotChassis() {
}
}
