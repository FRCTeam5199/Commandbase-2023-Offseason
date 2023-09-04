package com.frc.robot.subsystems.piecemanipulation;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.frc.robot.Constants;
import com.frc.robot.AbstractMotorInterfaces.SparkMotorController;

public class ArmSubsystem extends SubsystemBase {
    public SparkMotorController rotateMotorController;
    public SparkMotorController extendMotorController;
    PIDController rotatePIDController;
    PIDController extendPIDController;

	public ArmSubsystem() {}

	public void init() {
		motorInit();
		// if (!Constants.ARM_ELEVATOR_MANUAL) {
        	PIDInit();
		// }

		extendMotorController.resetEncoder();
		rotateMotorController.resetEncoder();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		// if (!Constants.ARM_ELEVATOR_MANUAL) {
			rotateMotorController.moveAtPercent(rotatePIDController.calculate(rotateMotorController.getRotations()));
			extendMotorController.moveAtPercent(extendPIDController.calculate(extendMotorController.getRotations()));
		// }
			System.out.println(isFront());
		// System.out.println("Rotations: " + rotateMotorController.getRotations());
		// System.out.println("Rotations: " + extendMotorController.getRotations());
		// System.out.println("PID: " + extendPIDController.calculate(extendMotorController.getRotations()));
		// System.out.println("Setpoint: " + extendPIDController.getSetpoint());
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

	public void motorInit() {
		rotateMotorController = new SparkMotorController(Constants.ARM_ROTATE_MOTOR_ID, MotorType.kBrushless);
		extendMotorController = new SparkMotorController(Constants.ARM_EXTEND_MOTOR_ID, MotorType.kBrushed);

        rotateMotorController.setBrake(true);
        extendMotorController.setBrake(true);
	}

    public void PIDInit() {
        rotatePIDController = new PIDController(Constants.ARM_ROTATE_PID.P, Constants.ARM_ROTATE_PID.I, Constants.ARM_ROTATE_PID.D);
        extendPIDController = new PIDController(Constants.ARM_EXTEND_PID.P, Constants.ARM_EXTEND_PID.I, Constants.ARM_EXTEND_PID.D);
    }
	
    public Command resetRotateEncoder() {
		return this.runOnce(() -> rotateMotorController.resetEncoder());
	}

	public Command resetExtendEncoder() {
		return this.runOnce(() -> extendMotorController.resetEncoder());
	}
	
	public Command setRotateSetpoint(int setpoint) {
		return this.runOnce(() -> rotatePIDController.setSetpoint(setpoint));
	}

	public Command setExtendSetpoint(int setpoint) {
		return this.runOnce(() -> extendPIDController.setSetpoint(setpoint));
	}
    /**
	 * Moves the Arm Rotate by a percent between -1 and 1 and stops it when finished.
	 */
	public Command moveRotate(float percent) {
		return this.runEnd(() -> rotateMotorController.moveAtPercent(percent), () -> rotateMotorController.moveAtPercent(0));
	}

	/**
	 * Moves the Arm Extend by a percent between -1 and 1 and stops it when finished.
	 */
	public Command moveExtend(float percent) {
		return this.runEnd(() -> extendMotorController.moveAtPercent(percent), () -> extendMotorController.moveAtPercent(0));
	}

	public void rotateFront() {
		rotatePIDController.setSetpoint(0);
	}

	public void rotateBack() {
		rotatePIDController.setSetpoint(30);
	}

	public void rotateHumanplayer() {
		rotatePIDController.setSetpoint(35);
	}

	public void rotateHigh() {
		rotatePIDController.setSetpoint(-105);
	}

	public void rotateMedium() {
		rotatePIDController.setSetpoint(-87);
	}

	public void rotateLow() {
		rotatePIDController.setSetpoint(-120);
	}

	public void extendMedium() {
		extendPIDController.setSetpoint(4.5);
	}

	public void extendHumanplayer() {
		extendPIDController.setSetpoint(10);
	}

	public void extend() {
		extendPIDController.setSetpoint(23);
	}

	public void retract() {
		extendPIDController.setSetpoint(5);
	}

	public boolean isRetracted() {
		return extendMotorController.getRotations() < 6;
	}

    public boolean isFront() {
		return rotateMotorController.getRotations() > -5;
	}
}