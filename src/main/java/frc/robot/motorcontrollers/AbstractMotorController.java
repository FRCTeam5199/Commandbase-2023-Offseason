package frc.robot.motorcontrollers;

import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.motorcontrollers.followers.AbstractFollowerMotorController;
import frc.robot.motorcontrollers.followers.SparkMotorFollower;
import frc.robot.motorcontrollers.followers.TalonMotorFollower;

import java.util.ArrayList;

public abstract class AbstractMotorController {

    SparkMotorController SparkMotorController;
    TalonMotorController TalonMotorController;
    VictorMotorController VictorMotorController;
    public static final ArrayList<AbstractMotorController> motors = new ArrayList<>();

    public double sensorToRealDistanceFactor;

    public abstract AbstractMotorController setInverted(boolean invert);

    public abstract void resetEncoder();

    public abstract AbstractMotorController setPid(PID pid);

    public abstract void moveAtVelocity(double amount);

    public abstract int getID();

    public abstract void moveAtPosition(double pos);

    public abstract void moveAtVoltage(double voltIn);

    public abstract void moveAtPercent(double percent);

    public abstract void setBrake(boolean brake);

    public abstract double getRotations();



    public abstract double getSpeed();

    public void setSensorToRealDistanceFactor(double s2rf) {
        sensorToRealDistanceFactor = s2rf;
    }

    public abstract double getVoltage();

    public abstract double getCurrent();

    public abstract AbstractMotorController setCurrentLimit(int limit);

    public static void resetAllMotors() {
        for (AbstractMotorController motor : motors) {
            motor.moveAtPercent(0);
        }

    }

    protected AbstractMotorController() {
        motors.add(this);
    }


    public abstract AbstractMotorController follow(AbstractMotorController leader, boolean invert);

    public abstract void setOutPutRange(double min, double max);

    public abstract boolean isFailed();

    public abstract int getMaxRPM();


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
                    return new SparkMotorFollower(followerIDs);
                case TALON_FX:
                    return new TalonMotorFollower(canbus, followerIDs);
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
                    return new SparkMotorController(ID, CANSparkMaxLowLevel.MotorType.kBrushless);
                case TALON_FX:
                    return new TalonMotorController(ID, canbus);
                case VICTOR:
                    return new VictorMotorController(ID);
                case SERVO:
                default:
                    throw new IllegalArgumentException("I cannot make a motor of type " + name());
            }
        }

        public AbstractMotorController createMotorOfType(int ID) {
            return this.createMotorOfType("Canivore1", ID);
        }
    }
}

