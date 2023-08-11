package frc.robot.motorcontrol;

import frc.robot.misc.PID;

public abstract class AbstractMotorController {
    public Double sensorToRealDistanceFactor;
    public Double sensorToRealTimeFactor;
    protected boolean failureFlag = false;
    protected String potentialFix;
    protected boolean isOverheated;

    public abstract AbstractMotorController setInverted(boolean invert);

    public abstract AbstractMotorController follow(AbstractMotorController leader, boolean invert);

    /**
     * The name is a unique motor identifier that includes the motor type and the id
     *
     * @return A unique, identifiable name for this motor
    */
    public abstract String getName();

    /**
     * Sets current encoder position to be the zero position. If you are absolutely crazy and want to set the encoder to
     * an artifical position, create an abstract method in {@link AbstractMotorControllerler} that takes an position Then,
     * implement this method to call your overloaded method and pass a default of 0. But really dont do that please
     */
    public abstract void resetEncoder();

    public abstract double getVelocity();
    
    public abstract double getVoltage();

    public abstract void setVelocity(double velocity);

    public abstract void setPosition(double position);

    public abstract void setVolts(double voltage);

    public abstract void setPercent(double percent);

    public abstract AbstractMotorController setPID(PID pid);
    
    public abstract void setRealFactorFromMotorRPM(double r2rf, double t2tf);

    //time to max speed.
    public abstract AbstractMotorController setOpenLoopRampRate(double timeToMaxSpeed);

    public abstract AbstractMotorController setBrake(boolean brake);

    public abstract double getRotations();

    public abstract double getAbsoluteRotations();

    public abstract AbstractMotorController setCurrentLimit(int limit);
}