package frc.robot.motorcontrol;


public class SwerveController {
    public TalonFXController driver;
    public TalonFXController steer;

    
    public SwerveController(int DriverID, int SteeringID){
        driver = new TalonFXController(DriverID, "Canivore1");
        steer = new TalonFXController(SteeringID, "Canivore1");
    }
    
}
