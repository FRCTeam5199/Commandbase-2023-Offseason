package frc.robot.controller;

import edu.wpi.first.wpilibj.Joystick;


public abstract class Abstractcontroller {
    protected final Joystick controller;

    protected Abstractcontroller(Joystick controller) {
        this.controller = controller;
    }
}
