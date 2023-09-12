package com.frc.robot.controls.customcontrollers;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.ArrayList;
import java.util.List;

// CommandButtonPanel takes in many CommandGenericHIDs (what we use for the Arduinos in the button panels),
// and stores them in a List<> in order to reference them later on

public class CommandButtonPanel {
    // Each Arduino uses 1 port and can have 16 buttons
    List<CommandGenericHID> ports;
    List<Integer> portIDs;

    // Varargs constructor that initializes new ports and stores their port IDs
    public CommandButtonPanel(int... IDs) { 
        portIDs = new ArrayList<Integer>(IDs.length);
        ports = new ArrayList<CommandGenericHID>(IDs.length);

        for (int portID : IDs) { 
            this.ports.add(new CommandGenericHID(portID)); // Make new port for each portID given
            this.portIDs.add(portID);
        }
    }

    // References .button() Trigger of a port, accounting for if it already exists
    public Trigger button(int portID, int buttonID){

        int portIDIndex = this.portIDs.indexOf(portID); // Should be >= 0 if a port with given portID already exists in ports<>

        if (portIDIndex >= 0) {
            return this.ports.get(portIDIndex).button(buttonID); // References existing port's .button() Trigger
        } else {
            this.portIDs.add(portID); // If port doesn't exist yet, register new port in ports<>
            this.ports.add(new CommandGenericHID(portID));

            return this.ports.get(this.ports.size() - 1).button(buttonID); // Reference the .button() Trigger of the latest item added at end of ports<> list
        }
    }

    public Trigger button(int portID, ControllerInterfaces.IDiscreteInput buttonID){
        return this.button(portID, buttonID.getChannel());
    }
}
