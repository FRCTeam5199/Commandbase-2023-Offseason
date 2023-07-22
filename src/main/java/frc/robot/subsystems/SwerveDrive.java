package frc.robot.subsystems;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.motorcontrol.PID;
import frc.robot.motorcontrol.SwerveController;
import frc.robot.Constants;


public class SwerveDrive extends SubsystemBase {
    CommandXboxController controller;
    SwerveController frontL;
    SwerveController frontR;
    SwerveController backL;
    SwerveController backR;
    double rotation;

    public void init(){
        controller = new CommandXboxController(0);
        frontL = new SwerveController(1, 2);
        frontR = new SwerveController(3, 4);
        backL = new SwerveController(5, 6);
        backR = new SwerveController(7, 8);

        frontL.driver.setRealFactorFromMotorRPM(10 / 70.0 * (4 * Math.PI / 12), 1 / 60D);
        frontR.driver.setRealFactorFromMotorRPM(10 / 70.0 * (4 * Math.PI / 12), 1 / 60D);
        backR.driver.setRealFactorFromMotorRPM(10 / 70.0 * (4 * Math.PI / 12), 1 / 60D);
        backL.driver.setRealFactorFromMotorRPM(10 / 70.0 * (4 * Math.PI / 12), 1 / 60D);


        setDrivingPID(new PID(0.56,.056,0,5.13));
        setSteeringPID(new PID(.0013, .00001, 0));
    }

    public void drive(){
        rotation = controller.getRightX() * Constants.DriverConstants.ROTATION_SPEED;
        double x = controller.getLeftX();
        double y = controller.getLeftY();


        frontL.driver.setPercent(y);
        frontL.steer.setPosition(y*x);

        if(rotation > 0 || rotation < 0){
            if(rotation > 0) {
                frontL.steer.setPosition(90);
            }else frontL.steer.setPosition(-90);
            frontL.driver.setPercent(rotation);

        }


        frontR.driver.follow(frontL.driver, isTurning());
        frontR.steer.follow(frontL.steer, false);

        backL.driver.follow(frontL.driver, isTurning());
        backL.steer.follow(frontL.steer, false);

        backR.driver.follow(frontL.driver, isTurning());
        backR.steer.follow(frontL.steer, false);
        
    


    }

    public void setDrivingPID(PID pid){
        frontL.driver.setPID(pid);
        frontR.driver.setPID(pid);
        backL.driver.setPID(pid);
        backR.driver.setPID(pid);
        
    }

    public void setSteeringPID(PID pid){
        frontL.steer.setPID(pid);
        frontR.steer.setPID(pid);
        backL.steer.setPID(pid);
        backR.steer.setPID(pid);
    
    }

    public boolean isTurning(){
        return controller.getRightX() > 0 || controller.getRightX() < 0;
    }
}
