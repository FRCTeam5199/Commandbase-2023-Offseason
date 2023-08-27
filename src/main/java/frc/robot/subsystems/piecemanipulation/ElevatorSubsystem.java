package frc.robot.subsystems.piecemanipulation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PieceManipulation;
import frc.robot.motorcontrol.SparkMaxController;

public class ElevatorSubsystem extends SubsystemBase {
	SparkMaxController elevatorMotorController;
	PIDController elevatorPIDController;

	public ElevatorSubsystem() {}

	public void init() {
		motorInit();
        if (!PieceManipulation.ARM_ELEVATOR_MANUAL) {
			PIDInit();
		}

		elevatorMotorController.resetEncoder();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		elevatorMotorController.setPercent(elevatorPIDController.calculate(elevatorMotorController.getRotations()));
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

	public void motorInit() {
		elevatorMotorController = new SparkMaxController(Constants.MotorIDs.ELEVATOR_MOTOR_ID);
        elevatorMotorController.setBrake(true);
	}

    public void PIDInit() {
        if (!PieceManipulation.ENABLE_ELEVATOR) {
        	elevatorPIDController = new PIDController(PieceManipulation.ARM_ROTATE_PID.P, PieceManipulation.ARM_ROTATE_PID.I, PieceManipulation.ARM_ROTATE_PID.D);
		}
    }

	public Command resetEncoder() {
		return this.runOnce(() -> elevatorMotorController.resetEncoder());
	}

	public Command setSetpoint(int setpoint) {
		return this.runOnce(() -> elevatorPIDController.setSetpoint(setpoint));
	}

	/**
	 * Moves the Elevator by a percent between -1 and 1 and stops it when finished.
	 */
	public Command move(float percent) {
		return this.runEnd(() -> elevatorMotorController.setPercent(percent), () -> elevatorMotorController.setPercent(0));
	}
}
