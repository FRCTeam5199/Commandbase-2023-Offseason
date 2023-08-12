package frc.robot.customcontrollers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandButtonPanel {

    CommandGenericHID buttonPanel1;
    CommandGenericHID buttonPanel2;

    public CommandButtonPanel(int port1, int port2) {
        buttonPanel1 = new CommandGenericHID(port1);
        buttonPanel2 = new CommandGenericHID(port2);
    }

    public Trigger button(int port, int id) {
        return port == 2 ? buttonPanel1.button(id) : buttonPanel2.button(id);
    }

}
