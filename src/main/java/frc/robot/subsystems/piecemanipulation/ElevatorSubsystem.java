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
		PIDInit();
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
        elevatorPIDController = new PIDController(PieceManipulation.ARM_PID.P, PieceManipulation.ARM_PID.I, PieceManipulation.ARM_PID.D);
    }

	public Command resetEncoder() {
		return this.runOnce(() -> elevatorMotorController.resetEncoder());
	}

	public Command setElevatorSetpoint(int setpoint) {
		return this.runOnce(() -> elevatorPIDController.setSetpoint(setpoint));
	}

	/**
	 * Moves the Elevator by a percent between -1 and 1 and stops it when finished.
	 */
	public Command moveElevator(int percent) {
		return this.runEnd(() -> elevatorMotorController.setPercent(percent), () -> elevatorMotorController.setPercent(0));
	}
}
