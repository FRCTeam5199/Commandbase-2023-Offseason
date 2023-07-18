package frc.robot.motorcontrollers;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.*;

public class SparkMotorController extends AbstractMotorController{
    CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController myPid;

    public SparkMotorController(int ID){
        this(ID, MotorType.kBrushless);
    }

    public SparkMotorController(int ID, CANSparkMaxLowLevel.MotorType type){
        super();
        motor = new CANSparkMax(ID, type);
        if (type == CANSparkMaxLowLevel.MotorType.kBrushed) {
            encoder = motor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 560);
        } else {
            encoder = motor.getEncoder();
        }
        myPid = motor.getPIDController();
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
        return null;
    }

    @Override
    public void moveAtVelocity(double amount) {
        myPid.setReference(amount/ sensorToRealDistanceFactor, CANSparkMax.ControlType.kVelocity, 0);
    }

    @Override
    public int getID() {
        return motor.getDeviceId();
    }

    @Override
    public void moveAtPosition(double pos) {

    }

    @Override
    public void moveAtVoltage(double voltIn) {
        motor.setVoltage(voltIn);
    }

    @Override
    public void moveAtPercent(double percent) {
        motor.set(percent);
    }

    @Override
    public void setBrake(boolean brake) {

    }

    @Override
    public double getRotations() {
        return encoder.getPosition() * sensorToRealDistanceFactor;
    }

    @Override
    public double getSpeed() {
        return encoder.getVelocity();
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
        if(motor.setSmartCurrentLimit(limit) != REVLibError.kOk){
            System.out.println("could not set current limit");
        }
        return this;
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
