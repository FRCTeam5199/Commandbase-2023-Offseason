package com.frc.robot.controls.customcontrollers;

import edu.wpi.first.wpilibj.Joystick;
import com.frc.robot.controls.customcontrollers.CommandButtonPanel;
import com.frc.robot.controls.customcontrollers.DefaultControllerEnums.ButtonStatus;
import com.frc.robot.controls.customcontrollers.DefaultControllerEnums.RawCompassInput;
import com.frc.robot.controls.customcontrollers.DefaultControllerEnums.ResolvedCompassInput;

import java.util.function.Function;

import static com.frc.robot.controls.customcontrollers.ControllerInterfaces.*;


/**
 * for ANY CONTROLLER, put EVERY GET METHOD in here as well as in the proper class! This allows for the COMPLETE HOT
 * SWAPPING of controllers simply by changing the constructor used.
 *
 * @author jojo2357
 */
public abstract class BaseController {
    /**
     * This is the meta registrar of controllers. This is to prevent two different controller types from existing on the
     * same channel and to reduce memory impact by reducing redundant objects. To use, simply query the index
     * corresponding to the port desired and if null, create and set. Otherwise, verify controller type then use.
     *
     * @see #createOrGet(int, IValidController)
     */
    protected static final BaseController[] allControllers = new BaseController[6];
    private static final String GENERIC_ERROR_CLAUSE = "If you believe this is a mistake, please override the overloaded get in the appropriate class";
    protected final Joystick controller;
    private final int JOYSTICK_CHANNEL;

    protected BaseController(Integer channel) {
        controller = new Joystick(channel);
        JOYSTICK_CHANNEL = channel;
    }

    public static BaseController createOrGet(int channel, IValidController controllerType) throws ArrayIndexOutOfBoundsException, ArrayStoreException, UnsupportedOperationException {
        if (channel < 0 || channel >= 6)
            throw new ArrayIndexOutOfBoundsException("You cant have a controller with id of " + channel);
        return allControllers[channel] = controllerType.getConstructor().apply(channel);
    }

    @Deprecated
    public double get(int channel) {
        return controller.getRawAxis(channel);
    }

    public ButtonStatus get(IDiscreteInput n) {
        throw new UnsupportedOperationException("This controller does not support getting a button status. " + GENERIC_ERROR_CLAUSE);
    }

    public double get(IContinuousInput axis) {
        throw new UnsupportedOperationException("This controller does not support getting a continuous input. " + GENERIC_ERROR_CLAUSE);
    }

    public double getPositive(IContinuousInput axis) {
        throw new UnsupportedOperationException("This controller does not support getting a continuous input. " + GENERIC_ERROR_CLAUSE);
    }

    public boolean hatIsExactly(RawCompassInput direction) {
        throw new UnsupportedOperationException("This controller does not have a hat. " + GENERIC_ERROR_CLAUSE);
    }

    public boolean hatIs(ResolvedCompassInput direction) {
        throw new UnsupportedOperationException("This controller does not have a hat." + GENERIC_ERROR_CLAUSE);
    }

    public void rumble(double percent) {
        throw new UnsupportedOperationException("This controller does not support rumbling. " + GENERIC_ERROR_CLAUSE);
    }

    //should NOT print BaseController (i hope)
    @Override
    public String toString() {
        return this.getClass().getName() + " on channel " + JOYSTICK_CHANNEL;
    }

    /*public enum DefaultControllers implements IValidController {
        // BOP_IT_CONTROLLER(BopItBasicController::new),
        // DRUM_CONTROLLER(DrumTimeController::new),
        // JOYSTICK_CONTROLLER(JoystickController::new),
        // SIX_BUTTON_GUITAR_CONTROLLER(SixButtonGuitarController::new),
        // WII_CONTROLLER(WiiController::new),
        // BUTTON_PANEL(ButtonPanelController::new),
        // XBOX_CONTROLLER(XBoxController::new);
        BUTTON_PANEL(CommandButtonPanel::new)

        private final Function<Integer, BaseController> constructor;

        DefaultControllers(Function<Integer, BaseController> structor) {
            constructor = structor;
        }

        public Function<Integer, BaseController> getConstructor() {
            return constructor;
        }
    }*/

    public interface IValidController {
        Function<Integer, BaseController> getConstructor();
    }
}