package frc.robot.AbstractMotorInterfaces.followers;

import frc.robot.AbstractMotorInterfaces.AbstractMotorController;
import frc.robot.AbstractMotorInterfaces.TalonMotorController;

/**
 * This works to wrap Falcon500's and maybe some other motors
 */
public class TalonFollowerMotorController extends AbstractFollowerMotorController {
    public TalonFollowerMotorController(String bus, int... ids) {
        motors = new TalonMotorController[ids.length];
        for (int i = 0; i < ids.length; i++)
            motors[i] = new TalonMotorController(ids[i], bus);
    }

    @Override
    public void invert(boolean invert) {
        for (AbstractMotorController motor : motors)
            motor.setInverted(invert);
    }
}
