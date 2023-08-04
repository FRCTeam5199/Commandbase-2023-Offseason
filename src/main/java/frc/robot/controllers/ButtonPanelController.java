package frc.robot.controllers;

import frc.robot.controllers.ControllerEnums.ButtonPanelButtons;
import frc.robot.controllers.basecontrollers.BaseController;
import frc.robot.controllers.basecontrollers.ControllerInterfaces;
import frc.robot.controllers.basecontrollers.DefaultControllerEnums;
import frc.robot.Robot;

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
        if (button instanceof ButtonPanelButtons || button instanceof ControllerEnums.ButtonPanelTapedButtons || button instanceof ControllerEnums.ButtonPanelButtons2022 || button instanceof ControllerEnums.TCannonExtraButtons || button instanceof ControllerEnums.MidiController || button instanceof ControllerEnums.ButtonPanelButtonsPlacement2023 || button instanceof ControllerEnums.ButtonPanelButtonsElse2023 || Robot.robotSettings.PERMIT_ROUGE_INPUT_MAPPING)
            return DefaultControllerEnums.ButtonStatus.get(controller.getRawButton(button.getChannel()));
        throw new IllegalArgumentException("Wrong mapping. Expected an enum of type " + ButtonPanelButtons.class.toString() + " but got " + button.getClass().toString() + " instead");


    }
}