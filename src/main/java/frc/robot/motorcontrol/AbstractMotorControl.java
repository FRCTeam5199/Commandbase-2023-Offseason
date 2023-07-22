package frc.robot.motorcontrol;

public abstract class AbstractMotorControl {
    public Double sensorToRealDistanceFactor;
    public Double sensorToRealTimeFactor;
    
    public abstract AbstractMotorControl setInverted(boolean invert);

    public abstract AbstractMotorControl follow(AbstractMotorControl leader, boolean invert);
    public abstract int getID();

    public abstract double getVelocity();

    //postion in this case refers to what the encoder says
    public abstract double getPosition();
    
    public abstract double getVoltage();

    public abstract void setVelocity(double velocity);

    public abstract void setPosition(double position);

    public abstract void setVolts(double voltage);

    public abstract void setPercent(double percent);

    public abstract AbstractMotorControl setPID(PID pid);
    
    public abstract void setRealFactorFromMotorRPM(double r2rf, double t2tf);



    


    
}
