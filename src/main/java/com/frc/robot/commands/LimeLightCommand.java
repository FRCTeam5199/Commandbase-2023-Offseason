package com.frc.robot.commands;
import com.frc.robot.commands.DriveCommand.Controls;
import com.frc.robot.controls.ManualControls;
import com.frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.frc.robot.utility.LimelightManager;

public class LimeLightCommand extends CommandBase {
    
    public double x, y, z;
    public boolean a;
    private final Drivetrain drivetrain;
    LimelightManager limelight;
    Controls controls;
    XboxController controller = new XboxController(0);
    PIDController limepid;

    public LimeLightCommand(Drivetrain drivetrain, LimelightManager limelight, Controls controls) {
        addRequirements(drivetrain);
        addRequirements(limelight);
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.controls = controls;

        limepid = new PIDController(1, 0, 0);
    
    }




    @Override
    public void execute() {
        limelight.lightOn();
        y = limelight.getY();
        x = controls.driveX();
        y =(y * .05)-.5;
        
        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-x, limepid.calculate(y), 0, drivetrain.getGyroscopeRotation()));
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
