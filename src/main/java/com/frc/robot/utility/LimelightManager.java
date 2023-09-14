package com.frc.robot.utility;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightManager extends SubsystemBase {
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry valueX = limelight.getEntry("tx");
    NetworkTableEntry valueY = limelight.getEntry("ty");
    NetworkTableEntry valueZ = limelight.getEntry("tz");

    public Command getTape(){
        return 

    }


    
}
