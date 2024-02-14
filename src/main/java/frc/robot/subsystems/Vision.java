package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    public DifferentialDrivePoseEstimator poseEstimator;
    public Field2d field;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry botPoseEntry = table.getEntry("botpose_wpiblue"); // TODO if red use red
    double[] defaultBotPose = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    NetworkTableEntry targetpose_robotspace = table.getEntry("targetpose");

    public Vision(DifferentialDrivePoseEstimator poseEstimator, Field2d field) {
        this.poseEstimator = poseEstimator;
        this.field = field;

        table.getEntry("pipeline").setNumber(0);

    }

    public void periodic(){
        /*
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);


        // post to smart dashboard periodically
        SmartDashboard.putNumber("Limelight/X", x);
        SmartDashboard.putNumber("Limelight/Y", y);
        SmartDashboard.putNumber("Limelight/Area", area);

        double hasTarget = table.getEntry("tv").getDouble(0);
        SmartDashboard.putNumber("Limelight/target", tv.getDouble(0.0));

        if (hasTarget == 0) {
            return;
        }

        double[] botPoseArray = botPoseEntry.getDoubleArray(defaultBotPose);

        Rotation2d botRotation = new Rotation2d(botPoseArray[5]);

        Pose2d botPose = new Pose2d(botPoseArray[0], botPoseArray[1], botRotation);

        poseEstimator.addVisionMeasurement(botPose, Timer.getFPGATimestamp());
        */ 
    }

    public Command getOrientAprilTag(RobotChassis chassis) {

        return new RunCommand(

                () -> {
                    
                    if(tv.getDouble(0.0) > 0.9){
                        double[] rotationToTarget = targetpose_robotspace.getDoubleArray(defaultBotPose);
                        double yaw = rotationToTarget[5];
                        chassis.chassisToBearing(yaw);
                    }

                },
                chassis);

    }

}
