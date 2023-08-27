package frc.robot.subsystems.piecemanipulation;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motorcontrol.SparkMaxController;
import frc.robot.Constants.PieceManipulation;

public class ArmSubsystem extends SubsystemBase {
    public SparkMaxController rotateMotorController;
    public SparkMaxController extendMotorController;
    PIDController rotatePIDController;
    PIDController extendPIDController;

	public ArmSubsystem() {}

	public void init() {
		motorInit();
		if (!PieceManipulation.ARM_ELEVATOR_MANUAL) {
        	PIDInit();
		}

		extendMotorController.resetEncoder();
		rotateMotorController.resetEncoder();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if (!PieceManipulation.ARM_ELEVATOR_MANUAL) {
			rotateMotorController.setPercent(rotatePIDController.calculate(rotateMotorController.getRotations()));
			extendMotorController.setPercent(-extendPIDController.calculate(extendMotorController.getRotations()));
		}

		// System.out.println(armExtendMotorController.getRotations());
		// System.out.println(-armExtendPIDController.calculate(armExtendMotorController.getRotations()));
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

	public void motorInit() {
		rotateMotorController = new SparkMaxController(Constants.MotorIDs.ARM_ROTATE_MOTOR_ID, MotorType.kBrushless);
		extendMotorController = new SparkMaxController(Constants.MotorIDs.ARM_EXTEND_MOTOR_ID, MotorType.kBrushed);

        rotateMotorController.setBrake(true);
        extendMotorController.setBrake(true);
	}

    public void PIDInit() {
        rotatePIDController = new PIDController(PieceManipulation.ARM_ROTATE_PID.P, PieceManipulation.ARM_ROTATE_PID.I, PieceManipulation.ARM_ROTATE_PID.D);
        extendPIDController = new PIDController(PieceManipulation.ARM_EXTEND_PID.P, PieceManipulation.ARM_EXTEND_PID.I, PieceManipulation.ARM_EXTEND_PID.D);
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
		return this.runEnd(() -> rotateMotorController.setPercent(percent), () -> rotateMotorController.setPercent(0));
	}

	/**
	 * Moves the Arm Extend by a percent between -1 and 1 and stops it when finished.
	 */
	public Command moveExtend(float percent) {
		return this.runEnd(() -> extendMotorController.setPercent(percent), () -> extendMotorController.setPercent(0));
	}
}
