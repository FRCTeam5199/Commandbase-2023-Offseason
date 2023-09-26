package com.frc.robot.subsystems.piecemanipulation;

import com.frc.robot.CompConstants;
import com.frc.robot.AbstractMotorInterfaces.SparkMotorController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase{
    public static SparkMotorController wristMotorController;
    public static PIDController wristPIDController;

    public void init(){
        motorInit();
        if (!CompConstants.Piecemanipulation.WRIST_MANUAL) {
            PIDInit();
        }

        wristMotorController.resetEncoder();
    }

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
        // if (!Constants.Piecemanipulation.ENABLE_WRIST) {
		    // wristMotorController.moveAtPercent(wristPIDController.calculate(wristMotorController.getRotations()));
        // }
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

    public void motorInit(){
        wristMotorController = new SparkMotorController(CompConstants.Piecemanipulation.WRIST_MOTOR_ID, MotorType.kBrushed);
        //wrist.setCurrentLimit(2,40);
        wristMotorController.setCurrentLimit(40);
        wristMotorController.setBrake(true);

        wristMotorController.resetEncoder();
    }

    public void PIDInit() {
        wristPIDController = new PIDController(CompConstants.Piecemanipulation.WRIST_PID.P, CompConstants.Piecemanipulation.WRIST_PID.I, CompConstants.Piecemanipulation.WRIST_PID.D);
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
        //return this.runEnd(() -> wristMotorController.moveAtPercent(-0.7), () -> wristMotorController.moveAtPercent(0));
        wristMotorController.moveAtPercent(-0.7);
        //return null;
	}
    
    public void moveRight() {
        //return this.runEnd(() -> wristMotorController.moveAtPercent(0.7),() -> wristMotorController.moveAtPercent(0));
        wristMotorController.moveAtPercent(0.7);
        //return null;
	}
    
    public void stopRotation() {
        wristMotorController.moveAtPercent(0);
    }
}