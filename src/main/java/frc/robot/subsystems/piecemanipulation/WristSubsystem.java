package frc.robot.subsystems.piecemanipulation;

import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motorcontrol.AbstractMotorController;
import frc.robot.motorcontrol.SparkMaxController;

public class WristSubsystem extends SubsystemBase{
    public static AbstractMotorController wrist;


    public void init(){
        
        createMotors();
    }

    public void createMotors(){
        wrist = new SparkMaxController(frc.robot.Constants.MotorIDs.WRIST_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushed);
        //wrist.setCurrentLimit(2,40);
        wrist.setCurrentLimit(40);
        wrist.setBrake(true);
    }

    public CommandBase rotateWrist(int armRot, int wristRot){
        if(armRot >= -125){
            if(wristRot >= 0 && wristRot <= 4000){
                return this.run(() -> wrist.setVolts(0));
            }else{
                return this.run(() -> wrist.setVolts(-6));
            }
        }else {
            if(wristRot <= 4011 && wristRot >= 10){
                return this.run(() -> wrist.setVolts(0));
            }else{
                return this.run(() -> wrist.setVolts(6));
            }
        }
    }
}
