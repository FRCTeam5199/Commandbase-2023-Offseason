package frc.robot.subsystems.piecemanipulation;

import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PieceManipulation;
import frc.robot.motorcontrol.SparkMaxController;

public class WristSubsystem extends SubsystemBase{
    public static SparkMaxController wristMotorController;
    public static PIDController wristPIDController;

    public void init(){
        motorInit();
        PIDInit();
    }

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		wristMotorController.setPercent(wristPIDController.calculate(wristMotorController.getRotations()));
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

    public void motorInit(){
        wristMotorController = new SparkMaxController(frc.robot.Constants.MotorIDs.WRIST_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushed);
        //wrist.setCurrentLimit(2,40);
        wristMotorController.setCurrentLimit(40);
        wristMotorController.setBrake(true);

        wristMotorController.resetEncoder();
    }

    public void PIDInit() {
        wristPIDController = new PIDController(PieceManipulation.WRIST_PID.P, PieceManipulation.WRIST_PID.I, PieceManipulation.WRIST_PID.D);
    }

    public Command resetEncoder() {
		return this.runOnce(() -> wristMotorController.resetEncoder());
	}

    public Command setWristSetpoint(int setpoint) {
        return this.runOnce(() -> wristPIDController.setSetpoint(setpoint));
    }

    public Command moveWrist(int percent) {
        return this.runEnd(() -> wristMotorController.setPercent(percent), () -> wristMotorController.setPercent(0));
    }
}
