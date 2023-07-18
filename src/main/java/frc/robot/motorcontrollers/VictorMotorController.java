package frc.robot.motorcontrollers;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class VictorMotorController extends AbstractMotorController {
    VictorSPX victor;

    public VictorMotorController(int ID){
        super();
        victor = new VictorSPX(ID);

    }

    @Override
    public AbstractMotorController setInverted(boolean invert) {
        return null;
    }

    @Override
    public void resetEncoder() {

    }

    public int getID() {
        return victor.getDeviceID();
    }

    @Override
    public AbstractMotorController setPid(PID pid) {
        return this;
    }

    @Override
    public void moveAtVelocity(double amount) {

    }

    @Override
    public void moveAtPosition(double pos) {

    }

    @Override
    public void moveAtVoltage(double voltIn) {

    }

    @Override
    public void moveAtPercent(double percent) {

    }

    @Override
    public void setBrake(boolean brake) {

    }

    @Override
    public double getRotations() {
        return 0;
    }

    @Override
    public double getSpeed() {
        return 0;
    }

    @Override
    public double getVoltage() {
        return 0;
    }

    @Override
    public double getCurrent() {
        return 0;
    }

    @Override
    public AbstractMotorController setCurrentLimit(int limit) {
        return null;
    }

    @Override
    public AbstractMotorController follow(AbstractMotorController leader, boolean invert) {
        return leader;
    }

    @Override
    public void setOutPutRange(double min, double max) {

    }

    @Override
    public boolean isFailed() {
        return false;
    }

    @Override
    public int getMaxRPM() {
        return 0;
    }
}
