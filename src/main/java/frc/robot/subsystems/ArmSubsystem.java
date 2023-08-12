package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motorcontrol.SparkMaxController;
import frc.robot.Constants.PieceManipulation;

public class ArmSubsystem extends SubsystemBase {
    SparkMaxController armMotorController;
    PIDController armPIDController;

	public ArmSubsystem() {}

	public void init() {
		System.out.println("Arm - init()");

		motorInit();
        // PIDInit();
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

		armMotorController = new SparkMaxController(Constants.MotorIDs.ARM_MOTOR_ID);

        armMotorController.setBrake(true);
	}

    // public void PIDInit() {
    //     armPIDController = new PIDController(PieceManipulation.ARM_PID.P, PieceManipulation.ARM_PID.I, PieceManipulation.ARM_PID.D);
    // }
    
    // public void setArmSetpoint(int setpoint) {
    //     armPIDController.setSetpoint(setpoint);
    // }

    public CommandBase resetEncoder() {
		return this.runOnce(() -> armMotorController.resetEncoder());
	}


    /**
	 * Moves the Arm by a percent between -1 and 1 and stops it when finished.
	 */
	public Command moveArm(int percent) {
		
        // return this.runOnce(() -> armMotor.setPercent());
        System.out.println("ArmSubsystem.java - moveArm");
		return this.runEnd(() -> armMotorController.setPercent(percent), () -> armMotorController.setPercent(0));
	}

    // public Command moveArmToSetpoint() {
		
    //     // return this.runOnce(() -> armMotor.setPercent());
    //     System.out.println("ArmSubsystem.java - moveArm");
	// 	return this.runEnd(() -> armMotorController.setPercent(), () -> armMotorController.setPercent(0));
	// }
}
