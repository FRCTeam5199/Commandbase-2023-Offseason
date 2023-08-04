package frc.robot.controllers.basecontrollers;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * The lame and basic controller. Does it get any more simpleton than this?
 *
 * @see BaseController
 * @see DefaultControllerEnums.XboxAxes
 * @see DefaultControllerEnums.XBoxButtons
 */
public class XBoxController extends BaseController {
    private final boolean triggerFlag = false;

    /**
     * Creates a new Xbox Controller object on a specified usb port
     *
     * @param n the usb port that the controller is on
     */
    XBoxController(Integer n) {
        super(n);
    }

    /**
     * get the state of an xbox axis
     *
     * @param axis xbox controller axis to query
     * @return the state of inputted axis on a scale of [-1,1]
     * @see #get(ControllerInterfaces.IDiscreteInput)
     */
    @Override
    public double get(ControllerInterfaces.IContinuousInput axis) {
        if (axis instanceof DefaultControllerEnums.XboxAxes)
            if (Math.abs(controller.getRawAxis(axis.getChannel())) > ((DefaultControllerEnums.XboxAxes) axis).DEADZONE) //makes sure axis is outside of the deadzone
                return controller.getRawAxis(axis.getChannel());
            else
                return 0;
        else if (Constants.PieceManipulation.PERMIT_ROUGE_INPUT_MAPPING)
            return controller.getRawAxis(axis.getChannel());
        throw new IllegalArgumentException("Wrong mapping. Expected an enum of type " + DefaultControllerEnums.XboxAxes.class + " but got " + axis.getClass().toString() + " instead");
    }

    /**
     * Gets the status of a button on the xbox controller
     *
     * @param button the button to query
     * @return the status of queried button
     * @see #get(ControllerInterfaces.IContinuousInput)
     */
    @Override
    public DefaultControllerEnums.ButtonStatus get(ControllerInterfaces.IDiscreteInput button) {
        if (button instanceof DefaultControllerEnums.XBoxButtons || Constants.PieceManipulation.PERMIT_ROUGE_INPUT_MAPPING)
            return DefaultControllerEnums.ButtonStatus.get(controller.getRawButton(button.getChannel()));
        if (button instanceof DefaultControllerEnums.XBoxPOVButtons)
            return DefaultControllerEnums.ButtonStatus.get(controller.getPOV() == button.getChannel());
        throw new IllegalArgumentException("Wrong mapping. Expected an enum of type " + DefaultControllerEnums.XBoxButtons.class + " but got " + button.getClass().toString() + " instead");
    }

    @Override
    public boolean hatIsExactly(DefaultControllerEnums.RawCompassInput direction) {
        return direction.POV_ANGLE == controller.getPOV();
    }

    @Override
    public boolean hatIs(DefaultControllerEnums.ResolvedCompassInput direction) {
        return direction.containsAngle(controller.getPOV());
    }

    @Override
    public void rumble(double percent) {
        controller.setRumble(GenericHID.RumbleType.kLeftRumble, percent);
        controller.setRumble(GenericHID.RumbleType.kRightRumble, percent);
    }
}