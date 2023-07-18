package frc.robot.motorcontrollers.followers;

import frc.robot.motorcontrollers.AbstractMotorController;
import frc.robot.motorcontrollers.SparkMotorController;

public class SparkMotorFollower extends AbstractFollowerMotorController{

    public SparkMotorFollower(int... ids) {
        motors = new AbstractMotorController[ids.length];
        for (int i = 0; i < ids.length; i++)
            motors[i] = new SparkMotorController(ids[i]);
    }

    @Override
    public void invert(boolean invert) {
        for (AbstractMotorController motor : motors)
            motor.setInverted(invert);
    }
}


