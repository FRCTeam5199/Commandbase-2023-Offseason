package com.frc.robot.utility;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightManager extends SubsystemBase {
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry tv;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");



    //read values periodically

    //post to smart dashboard periodically
    public void printlimelight(){
        SmartDashboard.putNumber("LimelightX", getX());
        SmartDashboard.putNumber("LimelightY", getY());
        SmartDashboard.putBoolean("LimelightArea", getTarget());
    }

    public Command limelight(){
       return run(()-> printlimelight());
    }
    
    //pipelines

    public double getX(){
        return table.getEntry("tx").getDouble(0);
    }

    public double getY(){

        return table.getEntry("ty").getDouble(0);
    }

    public boolean getTarget(){
        return table.getEntry("tv").getDouble(0) == 1;

    }


    

    // public Command getTape() {
    //     return;
    // }


    
}
