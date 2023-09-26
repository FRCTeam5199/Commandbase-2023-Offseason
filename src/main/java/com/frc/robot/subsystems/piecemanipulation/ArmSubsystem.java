package com.frc.robot.subsystems.piecemanipulation;

import com.frc.robot.Constants;
import com.frc.robot.AbstractMotorInterfaces.SparkMotorController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

	public SparkMotorController rotateMotorController;
	public SparkMotorController extendMotorController;
	PIDController rotatePIDController;
	PIDController extendPIDController;

	private boolean isFront = true;
	public double rotateSetpoint = 0;
	public double rotateOffset = 0;
	public double extendSetpoint = 0;
	public double extendOffset = 0;
	
	private double dunkOffset = 0;

	public ArmSubsystem() {
	}

	public void init() {
		motorInit();
		PIDInit();

		extendMotorController.resetEncoder();
		rotateMotorController.resetEncoder();
	}

	// This method will be called once per scheduler run
	@Override
	public void periodic() {
		// if (!Constants.ARM_ELEVATOR_MANUAL) {
		rotateMotorController.moveAtPercent(rotatePIDController.calculate(rotateMotorController.getRotations(), this.rotateSetpoint + this.rotateOffset + this.dunkOffset));
		if (extendMotorController.getRotations() < 1000) {
			extendMotorController.moveAtPercent(extendPIDController.calculate(extendMotorController.getRotations(), this.extendSetpoint + this.extendOffset));
		} else {
			extendMotorController.moveAtPercent(5);
		}
		// }
	}

	// This method will be called once per scheduler run during simulation
	@Override
	public void simulationPeriodic() {}

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

	/**
	 * Moves the Arm Rotate by a percent between -1 and 1 and stops it when
	 * finished.
	 */
	public Command moveRotate(float percent) {
		return this.runEnd(() -> rotateMotorController.moveAtPercent(percent), () -> rotateMotorController.moveAtPercent(0));
	}

	/**
	 * Moves the Arm Extend by a percent between -1 and 1 and stops it when
	 * finished.
	 */
	public Command moveExtend(float percent) {
		return this.runEnd(() -> extendMotorController.moveAtPercent(percent), () -> extendMotorController.moveAtPercent(0));
	}

	/**
	 * Sets the setpoint of the Arm Rotate
	 */
	public Command setRotateSetpoint(int setpoint) {
		return this.runOnce(() -> this.extendSetpoint = setpoint);
	}

	/**
	 * Sets the setpoint of the Arm Extend
	 */
	public Command setExtendSetpoint(int setpoint) {
		return this.runOnce(() -> this.extendSetpoint = setpoint);
	}

	public void rotateStable() {
		// rotatePIDController.setSetpoint(0);
		this.rotateSetpoint = 0;
		this.isFront = true;
		// this.isHigh = false;
	}

	public void rotateHumanPlayer() {
		// rotatePIDController.setSetpoint(35);
		this.rotateSetpoint = 35;
		this.isFront = true;
		// this.isHigh = false;
	}

	public void rotateHigh() {
		// rotatePIDController.setSetpoint(-110);
		this.rotateSetpoint = -107;
		this.isFront = false;
		// this.isHigh = true;
	}

	public void rotateMedium() {
		// rotatePIDController.setSetpoint(-89);
		this.rotateSetpoint = -89;
		this.isFront = false;
		// this.isHigh = false;
	}

	public void rotateLow() {
		// rotatePIDController.setSetpoint(-120);
		this.rotateSetpoint = -120;
		this.isFront = false;
		// this.isHigh = false;
	}

	public void setDunk() {
		this.dunkOffset = -9;
	}
	
	public void resetDunk() {
		this.dunkOffset = 0;
	}

	public void extendMedium() {
		// extendPIDController.setSetpoint(2.5);
		this.extendSetpoint = 2.5;
		// this.isRetracted = false;
	}
	
	public void extendLow() {
		// extendPIDController.setSetpoint(4);
		this.extendSetpoint = 4;
		// this.isRetracted = true;
	}

	public void extendHumanPlayer() {
		// extendPIDController.setSetpoint(7);
		this.extendSetpoint = 5;
		// this.isRetracted = false;
	}

	public void extend() {
		// extendPIDController.setSetpoint(22.5);
		this.extendSetpoint = 21;
		// this.isRetracted = false;
	}

	public void retract() {
		// extendPIDController.setSetpoint(5);
		this.extendSetpoint = 5;
		// this.isRetracted = true;
	}

	public boolean isFront() {
		return this.isFront;
	}

	// public boolean isHigh() {
	// 	return this.isHigh;
	// }
	
	public Command changeRotateOffset(double offset) {
		return this.runOnce(() -> rotateOffset += offset);
	}

	public Command changeExtendOffset(double offset) {
		return this.runOnce(() -> rotateOffset += offset);
	}
}