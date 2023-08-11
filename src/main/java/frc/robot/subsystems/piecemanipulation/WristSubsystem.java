package frc.robot.subsystems.piecemanipulation;

import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motorcontrol.AbstractMotorController;
import frc.robot.motorcontrol.SparkMaxController;

public class WristSubsystem extends SubsystemBase{
    public AbstractMotorController wrist;



    public void createMotors(){
        wrist = new SparkMaxController(frc.robot.Constants.MotorIDs.WRIST_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushed);
        //wrist.setCurrentLimit(2,40);
        wrist.setCurrentLimit(40);
        wrist.setBrake(true);
    }
}
