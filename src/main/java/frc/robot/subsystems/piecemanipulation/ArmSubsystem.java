package frc.robot.subsystems.piecemanipulation;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motorcontrol.SparkMaxController;
import frc.robot.Constants.PieceManipulation;

public class ArmSubsystem extends SubsystemBase {
    SparkMaxController armRotateMotorController;
    SparkMaxController armExtendMotorController;
    PIDController armRotatePIDController;
    PIDController armExtendPIDController;

	public ArmSubsystem() {}

	public void init() {
		motorInit();
		if (!PieceManipulation.ARM_ELEVATOR_MANUAL) {
        	PIDInit();
		}

		armExtendMotorController.resetEncoder();
		armRotateMotorController.resetEncoder();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if (!PieceManipulation.ARM_ELEVATOR_MANUAL) {
			armRotateMotorController.setPercent(armRotatePIDController.calculate(armRotateMotorController.getRotations()));
			armExtendMotorController.setPercent(-armExtendPIDController.calculate(armExtendMotorController.getRotations()));
		}

		System.out.println(armExtendMotorController.getRotations());
		System.out.println(-armExtendPIDController.calculate(armExtendMotorController.getRotations()));
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

	public void motorInit() {
		armRotateMotorController = new SparkMaxController(Constants.MotorIDs.ARM_ROTATE_MOTOR_ID, MotorType.kBrushless);
		armExtendMotorController = new SparkMaxController(Constants.MotorIDs.ARM_EXTEND_MOTOR_ID, MotorType.kBrushed);

        armRotateMotorController.setBrake(true);
        armExtendMotorController.setBrake(true);
	}

    public void PIDInit() {
        armRotatePIDController = new PIDController(PieceManipulation.ARM_ROTATE_PID.P, PieceManipulation.ARM_ROTATE_PID.I, PieceManipulation.ARM_ROTATE_PID.D);
        armExtendPIDController = new PIDController(PieceManipulation.ARM_EXTEND_PID.P, PieceManipulation.ARM_EXTEND_PID.I, PieceManipulation.ARM_EXTEND_PID.D);
    }
	
    public Command resetRotateEncoder() {
		return this.runOnce(() -> armRotateMotorController.resetEncoder());
	}

	public Command resetExtendEncoder() {
		return this.runOnce(() -> armExtendMotorController.resetEncoder());
	}
	
	public Command setRotateSetpoint(int setpoint) {
		return this.runOnce(() -> armRotatePIDController.setSetpoint(setpoint));
	}

	public Command setExtendSetpoint(int setpoint) {
		return this.runOnce(() -> armExtendPIDController.setSetpoint(setpoint));
	}
    /**
	 * Moves the Arm Rotate by a percent between -1 and 1 and stops it when finished.
	 */
	public Command moveRotate(float percent) {
		return this.runEnd(() -> armRotateMotorController.setPercent(percent), () -> armRotateMotorController.setPercent(0));
	}

	/**
	 * Moves the Arm Extend by a percent between -1 and 1 and stops it when finished.
	 */
	public Command moveExtend(float percent) {
		return this.runEnd(() -> armExtendMotorController.setPercent(percent), () -> armExtendMotorController.setPercent(0));
	}
}
