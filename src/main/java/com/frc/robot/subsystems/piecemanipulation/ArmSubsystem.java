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
	private boolean isRetracted = true;
	private double setPointOffset = 0;
	public int armLocation;

	public ArmSubsystem() {
	}

	public void init() {
		motorInit();
		PIDInit();

		System.out.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> IIINNNNIIIITTT <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
		extendMotorController.resetEncoder();
		rotateMotorController.resetEncoder();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		// if (!Constants.ARM_ELEVATOR_MANUAL) {
		rotateMotorController.moveAtPercent(rotatePIDController.calculate(rotateMotorController.getRotations()));
		if (extendMotorController.getRotations() < 1000) {
			extendMotorController.moveAtPercent(extendPIDController.calculate(extendMotorController.getRotations()));
		} else {
			extendMotorController.moveAtPercent(5);
		}
		// }P
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
		rotatePIDController = new PIDController(Constants.ARM_ROTATE_PID.P, Constants.ARM_ROTATE_PID.I,
				Constants.ARM_ROTATE_PID.D);
		extendPIDController = new PIDController(Constants.ARM_EXTEND_PID.P, Constants.ARM_EXTEND_PID.I,
				Constants.ARM_EXTEND_PID.D);
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

	public void rotateStable() {
		rotatePIDController.setSetpoint(0);
		this.isFront = true;
	}

	public void rotateHumanPlayer() {
		rotatePIDController.setSetpoint(35);
		this.isFront = true;
	}

	public void rotateHigh() {
		rotatePIDController.setSetpoint(-110);
		this.isFront = false;
	}

	public void rotateMedium() {
		rotatePIDController.setSetpoint(-89);
		this.isFront = false;
	}

	public void rotateLow() {
		rotatePIDController.setSetpoint(-120);
		this.isFront = false;
	}

	public void extendMedium() {
		extendPIDController.setSetpoint(2.5);
		this.isRetracted = false;
	}
	
	public void extendLow() {
		extendPIDController.setSetpoint(4);
		this.isRetracted = true;
	}

	public void extendHumanPlayer() {
		extendPIDController.setSetpoint(7);
		this.isRetracted = false;
	}

	public void extend() {
		extendPIDController.setSetpoint(22.5);
		this.isRetracted = false;
	}

	public void retract() {
		extendPIDController.setSetpoint(5);
		this.isRetracted = true;
	}

	public boolean isFront() {
		return this.isFront;
	}

	public Command addToRotate() {
		return this.runOnce(() -> setPointOffset += 0.5);
	}

	public Command lessToRotate() {
		return this.runOnce(() -> setPointOffset -= 0.5);
	}

	public Command setLastPosition(int x) {
		return this.runOnce(()-> armLocation = x);
	}

}