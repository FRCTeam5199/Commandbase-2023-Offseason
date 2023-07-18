package frc.robot.controller;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.configs.DefaultConfig;

public class Xbox {
    Joystick controller = new Joystick(DefaultConfig.DRIVER_CONTROLLER_PORT);

    public double getLXaxis(){
        return controller.getRawAxis(0);
    }

    public double getLYaxis(){
        return controller.getRawAxis(1);
    }

    public double getRXaxis(){
        return controller.getRawAxis(4);
    }

    public double getRYaxis(){
        return controller.getRawAxis(5);
    }

    public boolean Ybutton(){
        return controller.getRawButton(4);
    }

    public boolean Xbutton(){
        return controller.getRawButton(3);
    }

    public boolean Bbutton(){
        return controller.getRawButton(2);
    }

    public boolean Abutton(){
        return controller.getRawButton(1);
    }



}
