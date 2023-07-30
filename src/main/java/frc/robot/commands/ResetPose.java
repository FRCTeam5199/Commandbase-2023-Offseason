package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class ResetPose extends CommandBase {
    Drive driver;

    public ResetPose(Drive drive){
        driver = drive;
        
        addRequirements(drive);
    }

    public void init(){
        driver.setPoseO();
    }

    
}
