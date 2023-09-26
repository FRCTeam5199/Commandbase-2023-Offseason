package com.frc.robot.commands;
import com.frc.robot.controls.ManualControls;
import com.frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.frc.robot.utility.LimelightManager;

public class LimeLightCommand extends CommandBase {
    
    public double x, y;
    public boolean a;
    private final Drivetrain drivetrain;
    LimelightManager limelight;

    public LimeLightCommand(Drivetrain drivetrain, LimelightManager limelight) {
        addRequirements(drivetrain);
        addRequirements(limelight);
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        x = limelight.getX();
        a = limelight.getTarget();
        
    }



    @Override
    public void execute() {
        limelight.lightOn();
        y = limelight.getY();
        y =(y * .05) -.5;
        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, -y, 0, drivetrain.getGyroscopeRotation()));

        if(limelight.getY() > 6 && limelight.getY() < 8){
            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0,drivetrain.getGyroscopeRotation()));
            limelight.lightOff();
        }
    }

    @Override
    public boolean isFinished(){
        return limelight.getY() > 0 && limelight.getY() < 11;
    }

    @Override
    public void end(boolean interrupted){
        limelight.lightOff();
    }



}
