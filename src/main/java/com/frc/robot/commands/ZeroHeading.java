package com.frc.robot.commands;

import com.frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroHeading extends CommandBase{

    private final Drivetrain drivetrain;
    private double yawOffsetDegrees;

    
    public ZeroHeading(Drivetrain drivetrain, double yawOffsetDegrees){
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;

        this.yawOffsetDegrees = yawOffsetDegrees;
    }

    public ZeroHeading(Drivetrain drivetrain){
        this(drivetrain, drivetrain.getRawGyroRotation().getDegrees());
    }

    @Override
    public void execute(){
        boolean straight = drivetrain.getRawGyroRotation().getDegrees() - yawOffsetDegrees > 1.5;

        double rotatex = 0.0;
        if(straight){
            rotatex = (drivetrain.getRawGyroRotation().getDegrees() - yawOffsetDegrees) * (0.035 / 1.3) * 2;

        }

        drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                0,
                0,
                rotatex,
                drivetrain.getGyroscopeRotation()));

    }

    @Override
    public boolean isFinished(){
        return drivetrain.getRawGyroRotation().getDegrees() >= 0 || drivetrain.getRawGyroRotation().getDegrees() <=0;
    }

}
