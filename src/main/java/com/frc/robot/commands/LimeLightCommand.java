package com.frc.robot.commands;
import com.frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.frc.robot.utility.LimelightManager;

public class LimeLightCommand extends CommandBase {
    
    double x, y, a;
    private final Drivetrain drivetrain;
    LimelightManager limelight;

    public LimeLightCommand(Drivetrain drivetrain, double x ,  double y, double a) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.x = x;
        this.y = y;
        this.a = a;
        
    } 
    
    public void getLimelightValues()  {
        limelight.limelightvalues(x, y, a);

    }

    public void execute() {
        
    }

}
