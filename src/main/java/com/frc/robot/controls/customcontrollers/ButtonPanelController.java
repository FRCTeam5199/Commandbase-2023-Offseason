package com.frc.robot.controls.customcontrollers;

import com.frc.robot.controls.customcontrollers.BaseController;
import com.frc.robot.controls.customcontrollers.ControllerInterfaces;
import com.frc.robot.controls.customcontrollers.DefaultControllerEnums;
import com.frc.robot.Robot;

/**
 * Our custom built button panel that has a bunch of levers and switches (lol jk it has buttons silly) that is pretty
 * basic
 *
 * @see BaseController
 * @see ButtonPanelButtons
 * @see DefaultControllerEnums.ButtonStatus
 */
public class ButtonPanelController extends BaseController {
    public ButtonPanelController(Integer n) {
        super(n);
    }

    /**
     * Gets the Raw button value and returns true if it is pressed when it is run
     */
    @Override
    public DefaultControllerEnums.ButtonStatus get(ControllerInterfaces.IDiscreteInput button) {
        if (button instanceof CommandControllerEnums.ButtonPanelButtonsPlacement2023 || button instanceof CommandControllerEnums.ButtonPanelButtonsElse2023 || button instanceof CommandControllerEnums.ButtonPanelButtonsPlacement20239199 || button instanceof CommandControllerEnums.ButtonPanelButtonsElse20239199)
            return DefaultControllerEnums.ButtonStatus.get(controller.getRawButton(button.getChannel()));
        throw new IllegalArgumentException("Wrong mapping. Expected an enum of type " + CommandControllerEnums.ButtonPanelButtonsPlacement2023.class.toString() + " but got " + button.getClass().toString() + " instead");


    }
}