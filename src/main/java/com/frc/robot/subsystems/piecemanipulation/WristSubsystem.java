package com.frc.robot.subsystems.piecemanipulation;

import com.frc.robot.Constants;
import com.frc.robot.AbstractMotorInterfaces.SparkMotorController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class WristSubsystem extends SubsystemBase{
    public static SparkMotorController wristMotorController;
    public static PIDController wristPIDController;

    public void init(){
        motorInit();
        if (!Constants.WRIST_MANUAL) {
            PIDInit();
        }

        wristMotorController.resetEncoder();
    }

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
        if (!Constants.ENABLE_WRIST) {
		    wristMotorController.moveAtPercent(wristPIDController.calculate(wristMotorController.getRotations()));
        }
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

    public void motorInit(){
        wristMotorController = new SparkMotorController(Constants.WRIST_MOTOR_ID, MotorType.kBrushed);
        //wrist.setCurrentLimit(2,40);
        wristMotorController.setCurrentLimit(40);
        wristMotorController.setBrake(true);

        wristMotorController.resetEncoder();
    }

    public void PIDInit() {
        wristPIDController = new PIDController(Constants.WRIST_PID.P, Constants.WRIST_PID.I, Constants.WRIST_PID.D);
    }

    public Command resetEncoder() {
		return this.runOnce(() -> wristMotorController.resetEncoder());
	}

    public Command setSetpoint(int setpoint) {
        return this.runOnce(() -> wristPIDController.setSetpoint(setpoint));
    }

    public Command move(float percent) {
        return this.runEnd(() -> wristMotorController.moveAtPercent(percent), () -> wristMotorController.moveAtPercent(0));
    }

    public void moveLeft() {
        wristMotorController.moveAtPercent(-0.7);
	}

    public void moveRight() {
        wristMotorController.moveAtPercent(0.7);
	}

    public void stopRotation() {
        wristMotorController.moveAtPercent(0);
    }

    public Command moveLeftManual(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> moveLeft()),
            new WaitCommand(0.5),
            new InstantCommand(() -> stopRotation())
        );
    }
    public Command moveRigthManual(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> moveRight()),
            new WaitCommand(0.5),
            new InstantCommand(() -> stopRotation())
        );
    }
}