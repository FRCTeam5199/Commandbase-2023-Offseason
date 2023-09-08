package com.frc.robot.misc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Timer;

public class Time extends SubsystemBase {
    Timer timer = new Timer();

    public Command timed(long milliseconds){
        return runOnce(()-> {
            try {
                timer.wait(milliseconds);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        });
    }
}
