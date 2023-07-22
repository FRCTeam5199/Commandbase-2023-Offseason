package frc.robot.motorcontrol;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.ctre.phoenixpro.hardware.*;

public class TalonFXController extends AbstractMotorControl {
    TalonFX talon;
    
    


    public TalonFXController(int ID, String canbus){
        super();
        talon = new TalonFX(ID, canbus);

    }

    @Override
    public AbstractMotorControl setInverted(boolean invert) {
        talon.setInverted(invert);

        return this;
        
    }

    @Override
    public int getID() {
        return talon.getDeviceID();
    }

    @Override
    public double getVelocity() {
        return talon.getSelectedSensorVelocity();
    }

    @Override
    public double getPosition() {
        return talon.getSelectedSensorPosition();
    }

    @Override
    public void setVelocity(double velocity) {
        talon.set(TalonFXControlMode.Velocity, velocity);
    }

    @Override
    public void setPosition(double position) {
        talon.set(ControlMode.Position, position);
    }

    @Override
    public void setVolts(double voltage) {
        ((WPI_TalonFX) talon).setVoltage(voltage);
    }



    @Override
    public void setPercent(double percent) {
        talon.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public AbstractMotorControl setPID(PID pid) {
        return this;
    }
    @Override
    public double getVoltage() {
        return talon.getBusVoltage();
    }

    @Override
    public AbstractMotorControl follow(AbstractMotorControl leader, boolean invert) {
        return leader;
    }

    @Override
    public void setRealFactorFromMotorRPM(double r2rf, double t2tf) {
        sensorToRealDistanceFactor = r2rf/ 2048;
        sensorToRealTimeFactor = t2tf * 60D * 10D / 1D;
    }



    
}
