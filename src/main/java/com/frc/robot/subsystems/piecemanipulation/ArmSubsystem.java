package com.frc.robot.subsystems.piecemanipulation;

import com.frc.robot.CompConstants;
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
	private boolean isRetracted = true;

	public ArmSubsystem() {
	}

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
		if (extendMotorController.getRotations() < 1000) {
			extendMotorController.moveAtPercent(-extendPIDController.calculate(extendMotorController.getRotations()));
		} else {
			extendMotorController.moveAtPercent(-5);
		}
		// }
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

	public void motorInit() {
		rotateMotorController = new SparkMotorController(CompConstants.Piecemanipulation.ARM_ROTATE_MOTOR_ID, MotorType.kBrushless);
		extendMotorController = new SparkMotorController(CompConstants.Piecemanipulation.ARM_EXTEND_MOTOR_ID, MotorType.kBrushed);

		rotateMotorController.setBrake(true);
		extendMotorController.setBrake(true);
	}

	public void PIDInit() {
		rotatePIDController = new PIDController(CompConstants.Piecemanipulation.ARM_ROTATE_PID.P, CompConstants.Piecemanipulation.ARM_ROTATE_PID.I,
				CompConstants.Piecemanipulation.ARM_ROTATE_PID.D);
		extendPIDController = new PIDController(CompConstants.Piecemanipulation.ARM_EXTEND_PID.P, CompConstants.Piecemanipulation.ARM_EXTEND_PID.I,
				CompConstants.Piecemanipulation.ARM_EXTEND_PID.D);
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
	 * Moves the Arm Rotate by a percent between -1 and 1 and stops it when
	 * finished.
	 */
	public Command moveRotate(float percent) {
		return this.runEnd(() -> rotateMotorController.moveAtPercent(percent),
				() -> rotateMotorController.moveAtPercent(0));
	}

	/**
	 * Moves the Arm Extend by a percent between -1 and 1 and stops it when
	 * finished.
	 */
	public Command moveExtend(float percent) {
		return this.runEnd(() -> extendMotorController.moveAtPercent(percent),
				() -> extendMotorController.moveAtPercent(0));
	}

	public void rotateHumanPlayer() {
		rotatePIDController.setSetpoint(CompConstants.Setpoints.ARM_ROTATE_SETPOINT_HUMANPLAYER);
		this.isFront = true;
	}

	public void rotateStable() {
		rotatePIDController.setSetpoint(CompConstants.Setpoints.ARM_ROTATE_SETPOINT_STABLE);
		this.isFront = true;
	}

	public void rotateHigh() {
		rotatePIDController.setSetpoint(CompConstants.Setpoints.ARM_ROTATE_SETPOINT_HIGH);
		this.isFront = false;
	}

	public void rotateMid() {
		rotatePIDController.setSetpoint(CompConstants.Setpoints.ARM_ROTATE_SETPOINT_MID);
		this.isFront = false;
	}

	public void rotateLow() {
		rotatePIDController.setSetpoint(CompConstants.Setpoints.ARM_ROTATE_SETPOINT_LOW);
		this.isFront = false;
	}
	
	public void extendHumanPlayer() {
		extendPIDController.setSetpoint(CompConstants.Setpoints.ARM_EXTEND_SETPOINT_HUMANPLAYER);
		this.isRetracted = false;
	}
	
	public void extendStable() {
		extendPIDController.setSetpoint(CompConstants.Setpoints.ARM_EXTEND_SETPOINT_STABLE);
		this.isRetracted = true;
	}

	public void extendHigh() {
		extendPIDController.setSetpoint(CompConstants.Setpoints.ARM_EXTEND_SETPOINT_HIGH);
		this.isRetracted = false;
	}

	public void extendMid() {
		extendPIDController.setSetpoint(CompConstants.Setpoints.ARM_EXTEND_SETPOINT_MID);
		this.isRetracted = false;
	}

	public boolean isFront() {
		return this.isFront;
	}

}