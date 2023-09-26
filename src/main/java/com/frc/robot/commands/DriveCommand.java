package com.frc.robot.commands;

import com.frc.robot.subsystems.Drivetrain;
import com.frc.robot.utility.LimelightManager;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.frc.robot.commands.LimeLightCommand;

public class DriveCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final LimelightManager limelightManager = new LimelightManager();
    private final LimeLightCommand limelight;

    /**
     * Interface for defining the controls for the drive command.
     */
    public static interface Controls {
        double driveX();

        boolean rT();

        boolean lThrottle();

        double driveY();

        double driveRotationX();

        double driveRotationY();

        boolean driveResetYaw();

        boolean driveResetGlobalPose();
    }

    private Controls controls;

    private PIDController driveRotationPIDController = new PIDController(0.5, 0.05, 0.0);
    private PIDController driveTranslationYPIDController = new PIDController(0.13, 0.02, 0.0);
    private PIDController driveTranslationXPIDController = new PIDController(0.13, 0.02, 0.0);



    /**
     * Creates a new `DriveCommand` instance.
     *
     * @param drivetrainSubsystem the `Drivetrain` subsystem used by the command
     * @param controls            the controls for the command
     */
    public DriveCommand(Drivetrain drivetrainSubsystem, Controls controls) {
        this.drivetrain = drivetrainSubsystem;
        this.controls = controls;
        this.limelight = new LimeLightCommand(drivetrain, limelightManager);


        addRequirements(drivetrainSubsystem);

        driveRotationPIDController.setIntegratorRange(-0.5, 0.5);
        // driveRotationPIDController.setTolerance(3.5);
        driveRotationPIDController.setTolerance(0.0);
        driveRotationPIDController.enableContinuousInput(-180.0, 180.0);

        driveTranslationYPIDController.setIntegratorRange(-0.6, 0.6);
        driveTranslationYPIDController.setTolerance(0.0);
        driveTranslationYPIDController.enableContinuousInput(-180.0, 180.0);

        driveTranslationXPIDController.setIntegratorRange(-0.6, 0.6);
        driveTranslationXPIDController.setTolerance(0.0);
        driveTranslationXPIDController.enableContinuousInput(-180.0, 180.0);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (controls.driveResetYaw()) {
            drivetrain.zeroGyroscope();
        }

        if (controls.driveResetGlobalPose())
            drivetrain.resetOdometry(new Pose2d());

        if(controls.rT()) {
            drivetrain.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            controls.driveX() / 10,
                            controls.driveY() / 10,
                            controls.driveRotationX(),
                            drivetrain.getGyroscopeRotationNoApriltags()));
        } else if (controls.lThrottle()) {
            limelight.execute();


        }else {
            drivetrain.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            controls.driveX(),
                            controls.driveY(),
                            controls.driveRotationX(),
                            drivetrain.getGyroscopeRotationNoApriltags()));
        }// perhaps use getRawGyroRotation() instead?
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }


}
