// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.misc.PID;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int RobotNum = 5199;
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton {

    public static final PIDFConfig xAutoPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig yAutoPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_SPEED        = 4.0;
    public static final double MAX_ACCELERATION = 2.0;

    public static final boolean APRIL_TAG_ENABLED = true;
  }

  public static final class Drivebase {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
}


  public static class OperatorConstants {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.5;
    public static final double LEFT_Y_DEADBAND = 0.5;
  }


  public static class MotorIDs {
    /*
        ID Section
        Declare all motor ID's here
        Try and keep it organized by subsystem please
    */
  
    // Elevator
    public static final int ELEVATOR_MOTOR_ID = 40;
    
    // Arm
    public static final int ARM_ROTATE_MOTOR_ID = 39;
    public static final int ARM_EXTEND_MOTOR_ID = 43;

    // Wrist
    public static final int WRIST_MOTOR_ID = 27;
    
    //Top Intake 
    public static final int INTAKE_IN_ID = 12;
    public static final int INTAKE_OUT_ID = 2;

    //Bottom Intake
    public static final int BottomIntakeMotor_ID = 54;

    public static final int SPIKE_IN_ID = 8;
    public static final int SPIKE_OUT_ID = 11;
  }

  public static class Pneumatics {
    // Pneumatics Control Module
    public static final int PCM_ID = 50;

    // Compressor
    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;

    // Claw
    public static final int CLAW_IN_ID = 9;
    public static final int CLAW_OUT_ID = 10;
  }

      //////////////////////////
      //  Controller Ports    //
      //////////////////////////
        public static final int XBOX_CONTROLLER_USB_SLOT = 0,
        XBOX_CONTROLLER_USB_SLOT_2 = 1,
        BUTTON_PANEL1 = 2,
        BUTTON_PANEL2= 3;

    public static class PieceManipulation {

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Elevator
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        public static final Boolean ARM_ELEVATOR_MANUAL = false;

        public static final Boolean ENABLE_ELEVATOR = false;
        public static final String ELEVATOR_MOTOR_CANBUS = "rio";
        public static final PID ELEVATORPID = new PID(0.1, 0, 0);

        /////////////////////////////////////////////
        //  VV  Elevator Physical Properties  VV  //
        ///////////////////////////////////////////

        public static final double ELEVATOR_GEARING = 1 / 9.0;
        public static final double ELEVATOR_SPROCKET_DIAMETER = 2; //in

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // ARM
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        public static final boolean ENABLE_ARM = true;
        public static final String ARM_MOTOR_CANBUS = "rio";
        public static final PID ARM_ROTATE_PID = new PID(0.01, 0, 0);
        public static final PID ARM_EXTEND_PID = new PID(0.03, 0.001, 0);

        ////////////////////////////////////////
        //  VV  ARM Physical Properties  VV  //
        ///////////////////////////////////////

        public static final double ARM_GEARING = (1 / 15D) * (28 / 52D) * (15 / 61D);
        public static final double ARM_SPROCKET_DIAMETER = 1;

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // INTAKE
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        public static final boolean ENABLE_INTAKE = true;
        public static final String INTAKE_MOTOR_CANBUS = "rio";
        public static final boolean INTAKE_MANUAL = true;
        
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //  Wrist
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        public static final boolean ENABLE_WRIST = true;
        public static String WRIST_MOTOR_CANBUS = "rio";
        public static boolean WRIST_MANUAL = true;
        public static final PID WRIST_PID = new PID(0.1, 0, 0);

        // Claw
        public static final boolean ENABLE_CLAW = true;
    }
}
