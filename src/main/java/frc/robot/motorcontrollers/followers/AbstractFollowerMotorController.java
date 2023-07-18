package frc.robot.motorcontrollers.followers;

import frc.robot.motorcontrollers.AbstractMotorController;
public abstract class AbstractFollowerMotorController {
    protected  AbstractMotorController[] motors;

    public abstract void invert(boolean invert);

    public int getBundleID(){return motors[0].getID();}

    public AbstractMotorController getMotor(int id){
        if (id > 0 && motors.length-1 < id){
            return motors[id];
        }
        return null;
    }

    public void follow(AbstractMotorController leader) {
        follow(leader, false);
    }

    public void follow(AbstractMotorController leader, boolean invert) {
        for (AbstractMotorController follower : motors)
            follower.follow(leader, invert);
    }

    public void setBrake(boolean brake) {
        for (AbstractMotorController motor : motors)
            motor.setBrake(brake);
    }

    public void setCurrentLimit(int limit) {
        for (AbstractMotorController motor : motors)
            motor.setCurrentLimit(limit);
    }

    public boolean failureFlag() {
        for (AbstractMotorController motor : motors)
            if (motor.isFailed())
                return true;
        return false;
    }





}
