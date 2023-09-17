package com.frc.robot.subsystems.piecemanipulation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.frc.robot.CompConstants;
import com.frc.robot.AbstractMotorInterfaces.SparkMotorController;

public class ElevatorSubsystem extends SubsystemBase {
	SparkMotorController elevatorMotorController;
	PIDController elevatorPIDController;

	public ElevatorSubsystem() {}

	public void init() {
		motorInit();
        if (!CompConstants.Piecemanipulation.ARM_ELEVATOR_MANUAL) {
			PIDInit();
		}

		elevatorMotorController.resetEncoder();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
        if (!CompConstants.Piecemanipulation.ARM_ELEVATOR_MANUAL) {
			elevatorMotorController.moveAtPercent(elevatorPIDController.calculate(elevatorMotorController.getRotations()));
		}
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

	public void motorInit() {
		elevatorMotorController = new SparkMotorController(CompConstants.Piecemanipulation.ELEVATOR_MOTOR_ID);
        elevatorMotorController.setBrake(true);
	}

    public void PIDInit() {
		elevatorPIDController = new PIDController(CompConstants.Piecemanipulation.ELEVATOR_PID.P, CompConstants.Piecemanipulation.ELEVATOR_PID.I, CompConstants.Piecemanipulation.ELEVATOR_PID.D);
    }

	public Command resetEncoder() {
		return this.runOnce(() -> elevatorMotorController.resetEncoder());
	}

	public Command setSetpoint(int setpoint) {
		return this.runOnce(() -> elevatorPIDController.setSetpoint(setpoint));
	}

	public void humanPlayer() {
		elevatorPIDController.setSetpoint(CompConstants.Setpoints.ELEVATOR_SETPOINT_HUMANPLAYER);
	}
	
	public void high() {
		elevatorPIDController.setSetpoint(CompConstants.Setpoints.ELEVATOR_SETPOINT_HIGH);
	}
	
	public void mid() {
		elevatorPIDController.setSetpoint(CompConstants.Setpoints.ELEVATOR_SETPOINT_MID);
	}

	public void low() {
		elevatorPIDController.setSetpoint(CompConstants.Setpoints.ELEVATOR_SETPOINT_LOW);
	}

	/**
	 * Moves the Elevator by a percent between -1 and 1 and stops it when finished.
	 */
	public Command move(float percent) {
		return this.runEnd(() -> elevatorMotorController.moveAtPercent(percent), () -> elevatorMotorController.moveAtPercent(0));
	}
}