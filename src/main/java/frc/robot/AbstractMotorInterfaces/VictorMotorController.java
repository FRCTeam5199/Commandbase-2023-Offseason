package frc.robot.AbstractMotorInterfaces;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.misc.PID;
import frc.robot.Robot;

import static com.ctre.phoenix.motorcontrol.ControlMode.*;
import static com.ctre.phoenix.motorcontrol.NeutralMode.Brake;
import static com.ctre.phoenix.motorcontrol.NeutralMode.Coast;

/**
 * This works to wrap 775 Pros and maybe some other motors
 */
public class VictorMotorController extends AbstractMotorController {
    private final VictorSPX motor;

    public VictorMotorController(int id) {
        super();
        motor = new VictorSPX(id);

        sensorToRealDistanceFactor = 1 / frc.robot.Constants.CTRE_SENSOR_UNITS_PER_ROTATION;
        sensorToRealTimeFactor = 60D * 10D / 1D;
    }

    @Override
    public void setRealFactorFromMotorRPM(double r2rf, double t2tf) {
        sensorToRealDistanceFactor = r2rf / frc.robot.Constants.CTRE_SENSOR_UNITS_PER_ROTATION;
        sensorToRealTimeFactor = t2tf * 60D * 10D / 1D;
    }

    @Override
    public AbstractMotorController setInverted(boolean invert) {
        motor.setInverted(invert);
        return this;
    }

    @Override
    public String getName() {
        return "Victor: " + motor.getDeviceID();
    }

    @Override
    public int getID() {
        return motor.getDeviceID();
    }

    @Override
    public void setOutPutRange(double min, double max) {

    }

    @Override
    public AbstractMotorController follow(AbstractMotorController leader, boolean invert) {
        if (leader instanceof VictorMotorController) {
            motor.follow(((VictorMotorController) leader).motor);
        } else
            throw new IllegalArgumentException("I cant follow that!");
        setInverted(invert);

        this.sensorToRealTimeFactor = leader.sensorToRealTimeFactor;
        this.sensorToRealDistanceFactor = leader.sensorToRealDistanceFactor;
        return this;
    }

    @Override
    public AbstractMotorController unfollow() {
        motor.follow(motor);
        return this;
    }

    @Override
    public void resetEncoder() {
        if (motor.setSelectedSensorPosition(0) != ErrorCode.OK)
            if (!Robot.SECOND_TRY)
                throw new IllegalStateException("Victor Motor Controller with ID " + motor.getDeviceID() + " could not be reset");
            else
                failureFlag = true;
    }

    @Override
    public AbstractMotorController setPid(PID pid) {
        if (motor.config_kP(0, pid.getP()) != ErrorCode.OK || motor.config_kI(0, pid.getI()) != ErrorCode.OK || motor.config_kD(0, pid.getD()) != ErrorCode.OK || motor.config_kF(0, pid.getF()) != ErrorCode.OK)
            if (!Robot.SECOND_TRY)
                throw new IllegalStateException("Victor Motor Controller with ID " + motor.getDeviceID() + " PIDF could not be set");
            else
                failureFlag = true;
        return this;
    }

    @Override
    public void moveAtVelocity(double realVelocity) {
        if (isTemperatureAcceptable())
            motor.set(Velocity, realVelocity / sensorToRealDistanceFactor / sensorToRealTimeFactor);
        else
            motor.set(Velocity, 0);
    }

    @Override
    public void moveAtPosition(double pos) {
        motor.set(Position, pos / sensorToRealDistanceFactor);
    }

    @Override
    public double getVoltage() {
        return motor.getMotorOutputVoltage();
    }

    @Override
    public double getCurrent() {
        return -1;
    }

    @Override
    public void moveAtVoltage(double voltin) {
        //motor.set(VictorSPXControlMode.PercentOutput, voltin);
        throw new IllegalStateException("I can't do this loser");
    }

    @Override
    public AbstractMotorController setBrake(boolean brake) {
        motor.setNeutralMode(brake ? Brake : Coast);
        return this;
    }

    @Override
    public double getRotations() {
        return motor.getSelectedSensorPosition() * sensorToRealDistanceFactor;
    }

    @Override
    public double getSpeed() {
        return motor.getSelectedSensorVelocity() * sensorToRealDistanceFactor * sensorToRealTimeFactor;
    }

    //TODO make this work lol
    @Override
    public AbstractMotorController setCurrentLimit(int limit) {
        return this;
    }

    @Override
    public AbstractMotorController setCurrentLimit(int stallLimit, int freeLimit) {
        return null;
    }

    @Override
    public int getMaxRPM() {
        return SupportedMotors.VICTOR.MAX_SPEED_RPM;
    }

    @Override
    public void moveAtPercent(double percent) {
        if (isTemperatureAcceptable())
            motor.set(PercentOutput, percent);
        else
            motor.set(PercentOutput, 0);
    }

    @Override
    public AbstractMotorController setOpenLoopRampRate(double timeToMax) {
        if (motor.configOpenloopRamp(timeToMax) != ErrorCode.OK)
            if (!Robot.SECOND_TRY)
                throw new IllegalStateException("Victor Motor Controller with ID " + motor.getDeviceID() + " open loop ramp could not be set");
            else
                failureFlag = true;
        return this;
    }

    @Override
    public double getMotorTemperature() {
        return motor.getTemperature();
    }

    @Override
    public boolean isFailed() {
        Faults falts = new Faults();
        motor.getFaults(falts);
        return falts.hasAnyFault() || failureFlag;
    }

    @Override
    public String getSuggestedFix() {
        Faults foundFaults = new Faults();
        motor.getFaults(foundFaults);
        failureFlag = foundFaults.hasAnyFault();
        if (foundFaults.UnderVoltage) ;
            //report to PowerDistribution
        else if (foundFaults.RemoteLossOfSignal)
            potentialFix = "Ensure that motor %d is plugged into can AND power";
        else if (foundFaults.APIError)
            potentialFix = "Update the software for motor %d";
        else if (foundFaults.hasAnyFault())
            potentialFix = "Idk youre probably screwed";
        else
            potentialFix = "";
        return potentialFix;
    }
}
