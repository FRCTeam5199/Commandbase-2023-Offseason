package frc.robot.motorcontrol;

import static com.revrobotics.CANSparkMax.ControlType.kPosition;
import static com.revrobotics.CANSparkMax.ControlType.kVelocity;
import static com.revrobotics.CANSparkMax.IdleMode.kBrake;
import static com.revrobotics.CANSparkMax.IdleMode.kCoast;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.misc.PID;

import frc.robot.Robot;

public class SparkMaxController extends AbstractMotorControl {
    CANSparkMax sparkmax;
    SparkMaxPIDController myPid;
    RelativeEncoder encoder;

    public SparkMaxController(int channelID) {
        this(channelID, kBrushless);
    }

    public SparkMaxController(int channelID, CANSparkMaxLowLevel.MotorType type) {
        super();
        sparkmax = new CANSparkMax(channelID, type);
        if (type == CANSparkMaxLowLevel.MotorType.kBrushed) {
            encoder = sparkmax.getEncoder();
        } else {
            encoder = sparkmax.getEncoder();
        }
        myPid = sparkmax.getPIDController();
        //I dont know if talons do this or if we ever dont do this so here it is
        if (myPid.setOutputRange(-1, 1) != REVLibError.kOk)
            if (!Robot.SECOND_TRY)
                throw new IllegalStateException("Spark motor controller with ID " + sparkmax.getDeviceId() + " could not set its output range");
            else
                failureFlag = true;

        sensorToRealDistanceFactor = 1D;
        sensorToRealTimeFactor = 1D;
    }

    @Override
    public AbstractMotorControl setInverted(boolean invert) {
        sparkmax.setInverted(invert);

        return this;

    }

    @Override
    public AbstractMotorControl follow(AbstractMotorControl leader, boolean invert) {
        return leader;
    }

    @Override
    public String getName() {
        return "Spark: " + sparkmax.getDeviceId();
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public double getRotations() {
        return encoder.getPosition() * sensorToRealDistanceFactor;
    }

    public double getAbsoluteRotations() {
        return encoder.getPosition();
    }

    @Override
    public double getVoltage() {
        return sparkmax.getBusVoltage();
    }

    @Override
    public void setVelocity(double velocity) {
        myPid.setReference(velocity / sensorToRealDistanceFactor / sensorToRealTimeFactor, kVelocity, 0);
        
    }

    @Override
    public void setPosition(double position) {
        myPid.setReference(position * sensorToRealDistanceFactor, kPosition, 0);
        
    }

    @Override
    public void setVolts(double voltage) {
        sparkmax.setVoltage(voltage);
    }

    @Override
    public void setPercent(double percent) {
        // doesnt work with sparkMax
    }

    @Override
    public void setCurrentLimit(double currentLimit){
        sparkmax.setSmartCurrentLimit(currentLimit);
    }

    @Override
    public AbstractMotorControl setPID(PID pid) {
        if (myPid.setP(pid.getP(), 0) != REVLibError.kOk || myPid.setI(pid.getI(), 0) != REVLibError.kOk || myPid.setD(pid.getD(), 0) != REVLibError.kOk || myPid.setFF(pid.getF(), 0) != REVLibError.kOk)
            if (!Robot.SECOND_TRY)
                throw new IllegalStateException("Spark motor controller with ID " + sparkmax.getDeviceId() + " F in PIDF couldnt be reset");
            else
                failureFlag = true;
        return this;
    }

    @Override
    public void setRealFactorFromMotorRPM(double r2rf, double t2tf) {
        sensorToRealDistanceFactor = r2rf/ 2048;
        sensorToRealTimeFactor = t2tf * 60D * 10D / 1D;
    }

    @Override
    public AbstractMotorControl setOpenLoopRampRate(double timeToMaxSpeed) {
        if (sparkmax.setOpenLoopRampRate(timeToMaxSpeed) != REVLibError.kOk)
            if (!Robot.SECOND_TRY)
                throw new IllegalStateException("Spark motor controller with ID " + sparkmax.getDeviceId() + " could not set open loop ramp");
            else
                failureFlag = true;
        return this;
    }
    
    @Override
    public AbstractMotorControl setBrake(boolean brake) {
        sparkmax.setIdleMode(brake ? kBrake : kCoast);
        return this;
    }

    @Override
    public void resetEncoder() {
        if (encoder.setPosition(0) != REVLibError.kOk)
            if (!Robot.SECOND_TRY)
                throw new IllegalStateException("Spark motor controller with ID " + sparkmax.getDeviceId() + " could not reset its encoder");
            else
                failureFlag = true;
    }
}
