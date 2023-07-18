package frc.robot.motorcontrollers.followers;

import frc.robot.motorcontrollers.AbstractMotorController;
import frc.robot.motorcontrollers.TalonMotorController;

public class TalonMotorFollower extends AbstractFollowerMotorController{

    public TalonMotorFollower(String bus, int... ids){
        motors = new AbstractMotorController[ids.length];
        for(int i = 0; i < ids.length; i++){
            motors[i] = new TalonMotorController(ids[i], bus);
        }
    }
    @Override
    public void invert(boolean invert) {
        for (AbstractMotorController motor : motors)
            motor.setInverted(invert);
    }
}

