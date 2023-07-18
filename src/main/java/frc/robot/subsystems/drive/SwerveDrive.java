// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.DefaultConfig;
import frc.robot.motorcontrollers.AbstractMotorController;
import frc.robot.motorcontrollers.SwerveMotorController;
import frc.robot.motorcontrollers.PID;
import frc.robot.controller.Xbox;





public class SwerveDrive extends SubsystemBase
{
    SwerveMotorController swerveFL;
    SwerveMotorController swerveFR;
    SwerveMotorController swerveBL;
    SwerveMotorController swerveBR;







    Xbox controller = new Xbox();

    //Swerves


    //Swerve Positions in meters
    SwerveDriveKinematics swerveKin;
    Translation2d posFrontLeft = new Translation2d(0.2413, .3302);
    Translation2d posFrontRight = new Translation2d(-.2413, .3302);
    Translation2d posBackLeft = new Translation2d(.2413, -.3302);
    Translation2d posBackRight = new Translation2d(-.2413, -.3302);
    //PID





    public void init(){
        swerveKin = new SwerveDriveKinematics(posFrontLeft, posFrontRight, posBackLeft, posBackRight);
        setDrivingPID(new PID(.03,.02,.01));
        setSteeringPIDS(new PID(.03,.02,.01));
        swerves();



    }
    /**
     * Creates a new ExampleSubsystem.
     *
     * @return
     */
    public Runnable drive(double x, double y){
        double rotation = controller.getRXaxis() * DefaultConfig.ROTATION_SPEED;


        swerveFL.driver.moveAtPercent(y);
        swerveFL.steering.moveAtPosition(y*x);

        if(rotation > 0 || rotation < 0){
            if(rotation > 0) {
                swerveFL.steering.moveAtPosition(90);
            }else swerveFL.steering.moveAtPosition(-90);
            swerveFL.driver.moveAtPercent(rotation);

        }


        swerveFR.driver.follow(swerveFL.driver, false);
        swerveFR.steering.follow(swerveFL.steering, false);

        swerveBL.driver.follow(swerveFL.driver, isTurning());
        swerveBL.steering.follow(swerveFL.steering, false);

        swerveBR.driver.follow(swerveFL.driver, isTurning());
        swerveBR.steering.follow(swerveFL.steering, false);


        return null;
    }

    public boolean isTurning(){
        return controller.getRXaxis() > 0;
    }

    public void swerves(){
        swerveFL = new SwerveMotorController(1, AbstractMotorController.SupportedMotors.TALON_FX, 2, AbstractMotorController.SupportedMotors.TALON_FX);
        swerveFR = new SwerveMotorController(3, AbstractMotorController.SupportedMotors.TALON_FX, 4, AbstractMotorController.SupportedMotors.TALON_FX);
        swerveBL = new SwerveMotorController(5, AbstractMotorController.SupportedMotors.TALON_FX, 6, AbstractMotorController.SupportedMotors.TALON_FX);
        swerveBR = new SwerveMotorController(7, AbstractMotorController.SupportedMotors.TALON_FX, 8, AbstractMotorController.SupportedMotors.TALON_FX);



    }






    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */


    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }


    @Override
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }

    private void setSteeringPIDS(PID pid) {
    swerveFL.steering.setPid(pid);
    swerveFR.steering.setPid(pid);
    swerveBL.steering.setPid(pid);
    swerveBR.steering.setPid(pid);
}

    private void setDrivingPID(PID pid){
        swerveFL.driver.setPid(pid);
        swerveBL.driver.setPid(pid);
        swerveBR.driver.setPid(pid);

    }
}
