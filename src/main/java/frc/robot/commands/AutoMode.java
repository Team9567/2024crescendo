package frc.robot.commands;

import frc.robot.subsystems.RobotChassis;
import frc.robot.subsystems.RobotLauncher;


public class AutoMode {
    
    public RobotLauncher m_launcher;
    public RobotChassis m_Chassis;

    public AutoMode(RobotLauncher launcher, RobotChassis chassis) {

        this.m_launcher = launcher;
        this.m_Chassis = chassis;


        //addRequirements(launcher, chassis);
    }





}
 