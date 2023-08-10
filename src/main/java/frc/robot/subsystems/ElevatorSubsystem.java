package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motorcontrol.SparkMaxController;

public class ElevatorSubsystem extends SubsystemBase {
	SparkMaxController elevatorMotor;

	public ElevatorSubsystem() {}

	public void init() {
		System.out.println("Elevator - init()");

		motorInit();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

	public void motorInit() {
		System.out.println("Elevator - motorInit()");

		elevatorMotor = new SparkMaxController(Constants.MotorIDs.ELEVATOR_MOTOR_ID);
	}

	public CommandBase resetEncoder() {
		return this.runOnce(() -> elevatorMotor.resetEncoder());
	}

	/**
	 * Moves the Elevator by a percent between -1 and 1 and stops it when finished.
	 */
	public CommandBase moveElevator(int percent) {
		System.out.println("Elevator - moveElevator()");

		return this.runEnd(() -> elevatorMotor.setPercent(percent), () -> elevatorMotor.setPercent(0));
	}

// 	public CommandBase raiseElevator(int position) {
// 		System.out.println("Elevator - raiseElevator()");
// 		return this.runOnce(() -> elevatorMotor.setPosition(position));
// 	}

// 	public CommandBase lowerElevator(int position) {
// 		System.out.println("Elevator - lowerElevator()");
// 		return this.runOnce(() -> elevatorMotor.setPosition(position));
// 	}
}
