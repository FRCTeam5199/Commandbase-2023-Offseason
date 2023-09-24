package com.frc.robot.commands;
import com.frc.robot.controls.ManualControls;
import com.frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.frc.robot.utility.LimelightManager;

public class LimeLightCommand extends CommandBase {
    
    private double x, y;
    private boolean a;
    private final Drivetrain drivetrain;
    LimelightManager limelight;

    public LimeLightCommand(Drivetrain drivetrain, LimelightManager limelight) {
        addRequirements(drivetrain);
        addRequirements(limelight);
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.x = limelight.getX();
        this.y = limelight.getY();
        this.a = limelight.getTarget();
        
    } 
    


    public void execute() {
        
    }

}
