package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motorcontrol.SparkMaxController;

public class ArmSubsystem extends SubsystemBase {
    SparkMaxController armExtendMotor;
    SparkMaxController armRotateMotor;

	public ArmSubsystem() {}

	public void init() {
		System.out.println("Arm - init()");

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
		System.out.println("Arm - motorInit()");

		armExtendMotor = new SparkMaxController(Constants.MotorIDs.ARM_EXTEND_MOTOR_ID, MotorType.kBrushed);
		armRotateMotor = new SparkMaxController(Constants.MotorIDs.ARM_ROTATE_MOTOR_ID);
	}

    public CommandBase resetExtendEncoder() {
		return this.runOnce(() -> armExtendMotor.resetEncoder());
	}

    public CommandBase resetRotateEncoder() {
		return this.runOnce(() -> armRotateMotor.resetEncoder());
	}

    /**
	 * Moves the Arm Rotate by a percent between -1 and 1 and stops it when finished.
	 */
	public CommandBase moveArm(int percent) {
		System.out.println("Arm - moveElevator()");

		return this.runEnd(() -> armRotateMotor.setPercent(percent), () -> armRotateMotor.setPercent(0));
	}

    // public CommandBase raiseArm(int position) {
	// 	System.out.println("Elevator - raiseElevator()");
	// 	return this.runOnce(() -> elevatorMotor.setPosition(position));
	// }

	// public CommandBase lowerArm(int position) {
	// 	System.out.println("Elevator - lowerElevator()");
	// 	return this.runOnce(() -> elevatorMotor.setPosition(position));
	// }

    // public CommandBase extendArm(int position) {
    // 	System.out.println("Elevator - raiseElevator()");
    // 	return this.runOnce(() -> elevatorMotor.setPosition(position));
    // }

    // public CommandBase retractArm(int position) {
    // 	System.out.println("Elevator - lowerElevator()");
    // 	return this.runOnce(() -> elevatorMotor.setPosition(position));
    // }
}
