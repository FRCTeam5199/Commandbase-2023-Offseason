package com.frc.robot.AbstractMotorInterfaces;

import com.revrobotics.*;
import com.frc.robot.Robot;
import com.frc.robot.utility.PID;

import static com.revrobotics.CANSparkMax.ControlType.kPosition;
import static com.revrobotics.CANSparkMax.ControlType.kVelocity;
import static com.revrobotics.CANSparkMax.IdleMode.kBrake;
import static com.revrobotics.CANSparkMax.IdleMode.kCoast;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

/**
 * This works to wrap Neo's and maybe some other motors
 */
public class SparkMotorController extends AbstractMotorController {
    public final CANSparkMax motor;
    private final SparkMaxPIDController myPid;
    private final RelativeEncoder encoder;

    public SparkMotorController(int channelID) {
        this(channelID, kBrushless);
    }

    public SparkMotorController(int channelID, CANSparkMaxLowLevel.MotorType type) {
        super();
        motor = new CANSparkMax(channelID, type);
        if (type == CANSparkMaxLowLevel.MotorType.kBrushed) {
            encoder = motor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 560);
        } else {
            encoder = motor.getEncoder();
        }
        myPid = motor.getPIDController();
        //I dont know if talons do this or if we ever dont do this so here it is
        if (myPid.setOutputRange(-1, 1) != REVLibError.kOk)
            if (!Robot.SECOND_TRY)
                throw new IllegalStateException("Spark motor controller with ID " + motor.getDeviceId() + " could not set its output range");
            else
                failureFlag = true;

        sensorToRealDistanceFactor = 1D;
        sensorToRealTimeFactor = 1D;
    }

    @Override
    public AbstractMotorController setInverted(boolean invert) {
        motor.setInverted(invert);
        return this;
    }

    @Override
    public String getName() {
        return "Spark: " + motor.getDeviceId();
    }
    @Override
    public void resetEncoder() {
        if (encoder.setPosition(0) != REVLibError.kOk)
            if (!Robot.SECOND_TRY)
                throw new IllegalStateException("Spark motor controller with ID " + motor.getDeviceId() + " could not reset its encoder");
            else
                failureFlag = true;
    }

    @Override
    public AbstractMotorController setPid(PID pid) {
        if (myPid.setP(pid.getP(), 0) != REVLibError.kOk || myPid.setI(pid.getI(), 0) != REVLibError.kOk || myPid.setD(pid.getD(), 0) != REVLibError.kOk || myPid.setFF(pid.getF(), 0) != REVLibError.kOk)
            if (!Robot.SECOND_TRY)
                throw new IllegalStateException("Spark motor controller with ID " + motor.getDeviceId() + " F in PIDF couldnt be reset");
            else
                failureFlag = true;
        return this;
    }

    @Override
    public void moveAtVelocity(double velocityRPM) {
        //System.out.println("VelocityRPM " + velocityRPM);
        //System.out.println(this.sensorToRealDistanceFactor);
        myPid.setReference(velocityRPM / sensorToRealDistanceFactor / sensorToRealTimeFactor, kVelocity, 0);
    }

    @Override
    public void moveAtPosition(double pos) {
        myPid.setReference(pos * sensorToRealDistanceFactor, kPosition, 0);
    }

    @Override
    public void moveAtVoltage(double voltin) {
        motor.setVoltage(voltin);
    }

    @Override
    public AbstractMotorController setBrake(boolean brake) {
        motor.setIdleMode(brake ? kBrake : kCoast);
        return this;
    }

    @Override
    public double getRotations() {
        return encoder.getPosition() * sensorToRealDistanceFactor;
    }

    @Override
    public double getSpeed() {
        return encoder.getVelocity() * sensorToRealTimeFactor * sensorToRealDistanceFactor;
    }

    @Override
    public double getVoltage() {
        return motor.getBusVoltage() * motor.getAppliedOutput();
    }

    @Override
    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    @Override
    public AbstractMotorController setCurrentLimit(int limit) {
        if (motor.setSmartCurrentLimit(limit) != REVLibError.kOk)
            if (!Robot.SECOND_TRY)
                throw new IllegalStateException("Spark motor controller with ID " + motor.getDeviceId() + " could not set current limit");
            else
                failureFlag = true;
        return this;
    }

    @Override
    public AbstractMotorController setCurrentLimit(int stallLimit, int freeLimit) {
        if (motor.setSmartCurrentLimit(stallLimit, freeLimit) != REVLibError.kOk)
            if (!Robot.SECOND_TRY)
                throw new IllegalStateException("Spark motor controller with ID " + motor.getDeviceId() + " could not set current limit");
            else
                failureFlag = true;
        return this;
    }

    @Override
    public AbstractMotorController setOpenLoopRampRate(double timeToMax) {
        if (motor.setOpenLoopRampRate(timeToMax) != REVLibError.kOk)
            if (!Robot.SECOND_TRY)
                throw new IllegalStateException("Spark motor controller with ID " + motor.getDeviceId() + " could not set open loop ramp");
            else
                failureFlag = true;
        return this;
    }

    @Override
    public String getSuggestedFix() {
        short failmap = motor.getFaults();
        failureFlag = failmap != 0;
        if (failmap == 0)
            return "";
        //brownout
        if (((failmap) & 0x1) != 0)
            potentialFix = "Replace the battery";
            //overcurrent?
        else if (((failmap >> 1) & 0x1) != 0)
            potentialFix = "Unstall motor %d";
            //kIWDTReset???
        else if (((failmap >> 2) & 0x1) != 0)
            potentialFix = "¯\\_(ツ)_/¯";
            //kMotorFault
        else if (((failmap >> 3) & 0x1) != 0)
            potentialFix = "Motor fault";
        else if (((failmap >> 4) & 0x1) != 0)
            potentialFix = "Sensor fault";
        else if (((failmap >> 5) & 0x1) != 0)
            potentialFix = "Whoop whoop. Stall! Dont burn. Dont burn";
            //kEEPROMCRC
        else if (((failmap >> 6) & 0x1) != 0)
            potentialFix = "¯\\_(ツ)_/¯";
        else if (((failmap >> 7) & 0x3) != 0)
            potentialFix = "Check CAN connection";
            //kHasReset
        else if (((failmap >> 9) & 0x1) != 0)
            potentialFix = "Restart robocode";
            //kDRVFault
        else if (((failmap >> 10) & 0x3) != 0)
            potentialFix = "¯\\_(ツ)_/¯";
            //kDRVFault
        else
            potentialFix = "¯\\_(ツ)_/¯";
        return potentialFix;
    }

    @Override
    public boolean isFailed() {
        return motor.getFaults() != 0 || failureFlag;
    }

    @Override
    public int getMaxRPM() {
        return SupportedMotors.CAN_SPARK_MAX.MAX_SPEED_RPM;
    }

    @Override
    public void moveAtPercent(double percent) {
        motor.set(percent);
    }

    @Override
    public AbstractMotorController unfollow() {
        motor.follow(motor);
        return this;
    }

    @Override
    public AbstractMotorController follow(AbstractMotorController leader, boolean invert) {
        if (!(leader instanceof SparkMotorController))
            throw new IllegalArgumentException("I cant follow that!!");
        if (motor.follow(((SparkMotorController) leader).motor, invert) != REVLibError.kOk)
            if (!Robot.SECOND_TRY)
                throw new IllegalStateException("Spark motor controller with ID " + motor.getDeviceId() + " could not follow the leader");
            else
                failureFlag = true;

        this.sensorToRealTimeFactor = leader.sensorToRealTimeFactor;
        this.sensorToRealDistanceFactor = leader.sensorToRealDistanceFactor;
        return this;
    }

    @Override
    public void setRealFactorFromMotorRPM(double r2rf, double t2tf) {
        sensorToRealDistanceFactor = r2rf;
        sensorToRealTimeFactor = t2tf;
    }

    @Override
    public double getMotorTemperature() {
        return motor.getMotorTemperature();
    }

    @Override
    public int getID() {
        return motor.getDeviceId();
    }

    @Override
    public void setOutPutRange(double min, double max){
        myPid.setOutputRange(min,max);
    }

    public void setAllowedClosedLoopError(double threshold) {
        myPid.setSmartMotionAllowedClosedLoopError(threshold, 0);
    }

    public double getAbsoluteRotations() {
        return encoder.getPosition();
    }
}
