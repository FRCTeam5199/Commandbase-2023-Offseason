package frc.robot.motorcontrollers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Robot;

public class TalonMotorController extends AbstractMotorController {
    WPI_TalonFX talon;

    public TalonMotorController(int ID, String bus){
        super();
        talon = new WPI_TalonFX(ID, bus);
        sensorToRealDistanceFactor = 1;
    }


    @Override
    public AbstractMotorController setInverted(boolean invert) {
        return null;
    }

    @Override
    public void resetEncoder() {

    }

    @Override
    public AbstractMotorController setPid(PID pid) {
        return this;
    }

    @Override
    public void moveAtVelocity(double velocity) {
        talon.set(ControlMode.Velocity, velocity);
    }

    @Override
    public int getID() {
        return talon.getDeviceID();
    }

    @Override
    public void moveAtPosition(double pos) {
        talon.set(ControlMode.Position, pos);
    }

    @Override
    public void moveAtVoltage(double voltIn) {
        talon.set(ControlMode.Current, voltIn);

    }

    @Override
    public void moveAtPercent(double percent) {
        talon.set(ControlMode.PercentOutput, percent);
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
    public AbstractMotorController setCurrentLimit(int limit){
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
