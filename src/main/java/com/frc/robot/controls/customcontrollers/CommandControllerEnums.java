package com.frc.robot.controls.customcontrollers;

import java.util.function.Function;

import com.frc.robot.controls.customcontrollers.BaseController.IValidController;

public class CommandControllerEnums {

    public enum CustomControllers implements IValidController {
        BUTTON_PANEL_CONTROLLER(ButtonPanelController::new);

        private final Function<Integer, BaseController> constructor;

        CustomControllers(Function<Integer, BaseController> structor) {
            constructor = structor;
        }

        public Function<Integer, BaseController> getConstructor() {
            return constructor;
        }
    }
    
    public enum ButtonPanelButtonsPlacement2023 implements ControllerInterfaces.IDiscreteInput{
        //left to right and u can relable them later
        T1(11), T2(13), T3(12), T4(8), T5(9), T6(10),
        T7(4), T8(5), T9(6), SpikePickU(14), Stable(3);
        
        public final int AXIS_VALUE;

        ButtonPanelButtonsPlacement2023(int value) {
            this.AXIS_VALUE = value;
        }

        @Override
        public int getChannel() {
            return AXIS_VALUE;
        }
    }

    //needs to be in 
    public enum ButtonPanelButtonsElse2023 implements ControllerInterfaces.IDiscreteInput{
        Cone(12), Cube(13), Climb(8), GTStation1(11), GTStation2(14),
        SpikeD(9), SpikeU(10), GTShute(3), High(5),
        Mid(7), Low(6), Floor(4);

        public final int AXIS_VALUE;

        ButtonPanelButtonsElse2023(int value) {
            this.AXIS_VALUE = value;
        }

        @Override
        public int getChannel() {
            return AXIS_VALUE;
        }
    }

    public enum ButtonPanelButtonsPlacement20239199 implements ControllerInterfaces.IDiscreteInput{
        T1(1), T2(2), T3(3), T4(4), T5(5), T6(6),
        T7(7), T8(8), T9(9), Climb(10), HP1(11), HP2(12);

        public final int AXIS_VALUE;

        ButtonPanelButtonsPlacement20239199(int value) {
            this.AXIS_VALUE = value;
        }

        @Override
        public int getChannel() {
            return AXIS_VALUE;
        }

    }

    public enum ButtonPanelButtonsElse20239199 implements ControllerInterfaces.IDiscreteInput{
        Cone(1), Cube(2), SpikeU(3), SpikeD(4), Shute(5), 
        PickUp(6), Stable(7), Floor(8), High(9), Mid(10), Low(11);

        public final int AXIS_VALUE;

        ButtonPanelButtonsElse20239199(int value) {
            this.AXIS_VALUE = value;
        }

        @Override
        public int getChannel() {
            return AXIS_VALUE;
        }
    }

}
