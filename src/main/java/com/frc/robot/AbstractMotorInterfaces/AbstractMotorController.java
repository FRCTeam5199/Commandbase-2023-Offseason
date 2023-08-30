package com.frc.robot.AbstractMotorInterfaces;

import com.frc.robot.utility.PID;
import com.frc.robot.utility.UserInterface;
import com.frc.robot.AbstractMotorInterfaces.followers.AbstractFollowerMotorController;
import com.frc.robot.AbstractMotorInterfaces.followers.SparkFollowerMotorsController;
import com.frc.robot.AbstractMotorInterfaces.followers.TalonFollowerMotorController;
import com.frc.robot.Robot;

import java.util.ArrayList;

/**
 * This is the base class for any motor. It is not an interface because it has to have a {@link
 * #sensorToRealDistanceFactor nonstatic field} which is not doable in an interface. If you are to use a motor that we
 * did not implement, make a new class and use an Abstract motor
 *
 * @author jojo2357
 * @see AbstractFollowerMotorController
 * @see SparkMotorController
 * @see TalonMotorController
 * @see VictorMotorController
 */
public abstract class AbstractMotorController {
    public static final ArrayList<AbstractMotorController> motorList = new ArrayList<>();
    /**
     * Value to convert from sensor position to real units (this will vary between motors so know your units!)
     * Destination units are RPM that include the gearing on the motor
     * <p>
     * This is a Double (object, not value). If a requiring method is called and s2rf is not set, will throw a NPE, If
     * it is giving you hell and you dont want to actually fix the issue, just change to double (value, not object)
     */
    public Double sensorToRealDistanceFactor;
    public Double sensorToRealTimeFactor;
    protected boolean failureFlag = false;
    protected String potentialFix;
    protected boolean isOverheated;

    /**
     * Inverts the motor rotation from default for that motor (varies motor to motor of course)
     *
     * @param invert Whether to inver the motor or not
     * @return this object (factory style)
     */
    public abstract AbstractMotorController setInverted(boolean invert);

    /**
     * The name is a unique motor identifier that includes the motor type and the id
     *
     * @return A unique, identifiable name for this motor
     */
    public abstract String getName();

    /**
     * Sets current encoder position to be the zero position. If you are absolutely crazy and want to set the encoder to
     * an artifical position, create an abstract method in {@link AbstractMotorController} that takes an position Then,
     * implement this method to call your overloaded method and pass a default of 0. But really dont do that please
     */
    public abstract void resetEncoder();

    /**
     * You know how it is. Timeout is default per motor, channel defaults to 0 or motor default
     *
     * @param pid the {@link PID} object that contains pertinent pidf data
     * @return this object for factory style construction
     */
    public abstract AbstractMotorController setPid(PID pid);

    /**
     * @param amount requested drive velocity in RPM
     */
    public abstract void moveAtVelocity(double amount);

    /**
     * @param pos requested position
     */
    public abstract void moveAtPosition(double pos);

    public abstract void moveAtVoltage(double voltIn);

    /**
     * Sets the idle mode to either be (brake = false) minimally resistive or (brake = true) to resist all motion/use
     * ERF to slow motor (actual implemetation varies between motors)
     *
     * @param brake whether to apply idle resistance
     * @return this object for factory style construction
     */
    public abstract AbstractMotorController setBrake(boolean brake);

    /**
     * Gets the default encoder's position. Accounts for gearings but not wheelings unless ur crazy and accounted for
     * wheel size in {@link #sensorToRealDistanceFactor}
     *
     * @return Distance output shaft has moved in rotations
     */
    public abstract double getRotations();

    /**
     * Gets the current speed of the motor after taking into account {@link #sensorToRealDistanceFactor s2rf}.
     *
     * @return The current speed of the motor
     */
    public abstract double getSpeed();

    public abstract double getVoltage();

    public abstract double getCurrent();

    /**
     * Sets the maximum allowable current that will flow through this motor
     *
     * @param limit max current in amps
     * @return this object for factory style construction
     */
    public abstract AbstractMotorController setCurrentLimit(int limit);

    public abstract AbstractMotorController setCurrentLimit(int stallLimit, int freeLimit);

    /**
     * Sets the ramp rate for open loop control modes. This is the maximum rate at which the motor controller's output
     * is allowed to change in said mode.
     *
     * @param timeToMax time in seconds to go from 0 to full power
     * @return this object for factory style construction
     */
    public abstract AbstractMotorController setOpenLoopRampRate(double timeToMax);

    /**
     * Used for {@link frc.selfdiagnostics.ISimpleIssue issues} and gives a potential fix based on built in failure
     * reported issue
     *
     * @return A possible fix to what ails the motor
     */
    public abstract String getSuggestedFix();

    /**
     * Active failure check that uses motor built in failure checks to determine viability of the motor
     *
     * @return true if the motor is failed, false if nominal
     */
    public abstract boolean isFailed();

    public abstract int getMaxRPM();


    public void setOutPutRange() {

    }

    /**
     * Different from {@link #setSensorToRealDistanceFactor(double)} because this will first convert sensor units to
     * revolutions per second, and then apply the supplied conversion
     *
     * @param r2rf Conversion from motor RPS to real RPM, ft/s, etc
     * @param t2tf
     */
    public abstract void setRealFactorFromMotorRPM(double r2rf, double t2tf);

    /**
     * In order to prevent out of control PID loops from emerging, especially coming out of a disable in test mde, we
     * set all motors to idle. If we really want them to move then this method will take no effect because
     */
    public static void resetAllMotors() {
        for (AbstractMotorController motor : motorList) {
            motor.moveAtPercent(0);
        }
    }

    /**
     * Sets the motor output on a percent output basis
     *
     * @param percent -1 to 1 output requested
     */
    public abstract void moveAtPercent(double percent);

    protected AbstractMotorController() {
        motorList.add(this);
    }

    /**
     * Have this motor follow another motor (must be the same motor ie talon to talon). This motor will be the child and
     * the passed motor will be the leader
     *
     * @param leader motor to follow
     * @return this object for factory style construction
     * @see AbstractFollowerMotorController
     */
    public AbstractMotorController follow(AbstractMotorController leader) {
        return follow(leader, false);
    }

    /**
     *
     * @return this object for factory style construction
     * @see AbstractFollowerMotorController
     */
    public abstract AbstractMotorController unfollow();

    /**
     * Have this motor follow another motor (must be the same motor ie talon to talon). This motor will be the child and
     * the passed motor will be the leader
     *
     * @param leader motor to follow
     * @param invert whether to invert this follower
     * @return this object for factory style construction
     * @see AbstractFollowerMotorController
     */
    public abstract AbstractMotorController follow(AbstractMotorController leader, boolean invert);

    /**
     * see docs for {@link #sensorToRealDistanceFactor} for full explanation
     *
     * @param s2rf Conversion from encoder units to RPM including the gearing
     *
     * @see #setRealFactorFromMotorRPM(double, double)
     * @deprecated
     */
    @Deprecated
    public void setSensorToRealDistanceFactor(double s2rf) {
        sensorToRealDistanceFactor = s2rf;
    }

    @Deprecated
    public void setSensorToRealTimeFactor(double s2rf) {
        sensorToRealTimeFactor = s2rf;
    }

    /**
     * Self explanatory. Takes into account {@link #getMotorTemperature() motor temp} and {@link
     * frc.robot.robotconfigs.DefaultConfig#OVERHEAT_THRESHOLD} to determine if the motor is running too hot for usage
     *
     * @return invert {@link #isOverheated}
     */
    protected boolean isTemperatureAcceptable() {
        if (com.frc.robot.Constants.ENABLE_OVERHEAT_DETECTION) {
            if (getMotorTemperature() >= com.frc.robot.Constants.OVERHEAT_THRESHOLD) {
                if (!isOverheated) {
                    UserInterface.smartDashboardPutBoolean("OVERHEAT " + getID(), false);
                    isOverheated = true;
                }
            } //wait 5 degrees to unoverheat
            else if (isOverheated && getMotorTemperature() < com.frc.robot.Constants.OVERHEAT_THRESHOLD - 5) {
                isOverheated = false;
                UserInterface.smartDashboardPutBoolean("OVERHEAT " + getID(), true);
            }
            return !isOverheated;
        } else {
            return true;
        }
    }

    /**
     * Gets the temperature of the motor
     *
     * @return the temperature in celcius
     */
    public abstract double getMotorTemperature();

    /**
     * Gets the device id based on the motor's built in function. <b>DOES NOT STORE CONSTRUCTED MOTOR ID</b>
     *
     * @return the device id this object controls
     */
    public abstract int getID();

    public abstract void setOutPutRange(double min, double max);


    /**
     * This should be one-for-one replicated for each {@link AbstractMotorController motor controller} in order to
     * create settings to switch between motor implementations
     */
    public enum SupportedMotors {
        //Spark = Neo 550, Talon = Falcon 500, Victor = 775pros, Servo = whatever servo you put in. I didn't have a better place for this so it's here
        CAN_SPARK_MAX(11710), TALON_FX(6380), VICTOR(18730), SERVO;

        /**
         * Read the name!
         */
        public final int MAX_SPEED_RPM;

        SupportedMotors(int speed) {
            MAX_SPEED_RPM = speed;
        }

        SupportedMotors() {
            MAX_SPEED_RPM = 0;
        }

        public AbstractFollowerMotorController createFollowerMotorsOfType(String canbus, int... followerIDs) {
            switch (this) {
                case CAN_SPARK_MAX:
                    return new SparkFollowerMotorsController(followerIDs);
                case TALON_FX:
                    return new TalonFollowerMotorController(canbus, followerIDs);
                case VICTOR:
                case SERVO:
                default:
                    throw new IllegalArgumentException("I cannot make a motor follower of type " + name());
            }
        }

        public AbstractFollowerMotorController createFollowerMotorsOfType(int... followerIDs) {
            return this.createFollowerMotorsOfType("rio", followerIDs);
        }

        public AbstractMotorController createMotorOfType(String canbus, int ID) {
            switch (this) {
                case CAN_SPARK_MAX:
                    return new SparkMotorController(ID);
                case TALON_FX:
                    return new TalonMotorController(ID, canbus);
                case VICTOR:
                    return new VictorMotorController(ID);
                case SERVO:
                default:
                    throw new IllegalArgumentException("I cannot make a motor of type " + name() + " Howd you manage to fail at this L + ratio");
            }
        }

        public AbstractMotorController createMotorOfType(int ID) {
            return this.createMotorOfType("rio", ID);
        }
    }
}
