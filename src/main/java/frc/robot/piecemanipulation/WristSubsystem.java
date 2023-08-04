package frc.robot.piecemanipulation;

import com.revrobotics.CANSparkMaxLowLevel;

import frc.controllers.ControllerEnums;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;

import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;

import frc.motors.AbstractMotorController;
import frc.motors.SparkMotorController;
import frc.motors.TalonMotorController;

import static frc.robot.Robot.robotSettings;

public class Wrist implements ISubsystem {
    public AbstractMotorController wrist;
    public BaseController panel1, panel2, xbox2, midiTop, midiBot;

    public Wrist(){
        addToMetaList();
        init();
    }
    @Override
    public void init() {
        createControllers();
        createMotors();
        wrist.setPid(robotSettings.WRISTPID);
        wrist.setCurrentLimit(10);
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return null;
    }

    @Override
    public void updateTest() {

    }

    @Override
    public void updateTeleop() {
        /*if(robotSettings.ARM_ELEVATOR_MANUAL){
            if (midiTop.get(ControllerEnums.MidiController.R3C7) == DefaultControllerEnums.ButtonStatus.DOWN) {
                wrist.moveAtVoltage(-2);
            }
            else if (midiTop.get(ControllerEnums.MidiController.R2C8) == DefaultControllerEnums.ButtonStatus.DOWN) {
                wrist.moveAtVoltage(2);
            }
            else {
                wrist.moveAtVoltage(0);
            }
        }else {
            if (midiTop.get(ControllerEnums.MidiController.R2C7) == DefaultControllerEnums.ButtonStatus.DOWN) {
                wrist.moveAtPosition(-2);
            }
            else if (midiTop.get(ControllerEnums.MidiController.R2C8) == DefaultControllerEnums.ButtonStatus.DOWN) {
                wrist.moveAtPosition(-40.5);
            }
        }*/
        //System.out.println(wrist.getRotations());
        if (xbox2.get(DefaultControllerEnums.XBoxButtons.RIGHT_BUMPER) == DefaultControllerEnums.ButtonStatus.DOWN)
            wrist.resetEncoder();
        System.out.println(wrist.getRotations());
    }

    @Override
    public void updateAuton() {

    }

    @Override
    public void updateGeneric() {

    }

    @Override
    public void initTest() {

    }

    @Override
    public void initTeleop() {

    }

    @Override
    public void initAuton() {

    }

    @Override
    public void initDisabled() {

    }

    @Override
    public void initGeneric() {

    }

    @Override
    public String getSubsystemName() {
        return null;
    }

    public void createMotors(){
        if(robotSettings.WRIST_MOTOR_TYPE == AbstractMotorController.SupportedMotors.TALON_FX)
            wrist = new TalonMotorController(robotSettings.WRIST_MOTOR_ID, robotSettings.WRIST_MOTOR_CANBUS);
        if(robotSettings.WRIST_MOTOR_TYPE == AbstractMotorController.SupportedMotors.CAN_SPARK_MAX)
            wrist = new SparkMotorController(robotSettings.WRIST_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushed);
        //wrist.setCurrentLimit(2,40);
        wrist.setCurrentLimit(40);
        wrist.setBrake(true);
    }

    public void createControllers(){
        xbox2 = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT_2, BaseController.DefaultControllers.XBOX_CONTROLLER);
        midiTop = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_TOP_ID, BaseController.DefaultControllers.BUTTON_PANEL);
        midiBot = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_BOT_ID, BaseController.DefaultControllers.BUTTON_PANEL);
        panel1 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT1, BaseController.DefaultControllers.BUTTON_PANEL);
        panel2 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT2, BaseController.DefaultControllers.BUTTON_PANEL);
    }
}