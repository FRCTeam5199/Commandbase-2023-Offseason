// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frc.robot;


import com.frc.robot.utility.PID;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class CompConstants {

  public static final int robotNum = 5199;

  public static final int GYRO_ID = 22;

  

  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4953;
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.6477;

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 11;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(23);

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(206);

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 14;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(171);

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 5;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 6;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 13;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(245);

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
      SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
      SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public static final double MAX_ACCELERATION = 5.0; // m/s^2
  public static final double MAX_ANGULAR_ACCELERATION = 20.0; // rad/s^2

  public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
  public static final int OVERHEAT_THRESHOLD = 80;
  public static final boolean ENABLE_OVERHEAT_DETECTION = false;
  public static final boolean ENABLE_CAMERA = false;
  public static final String DRIVE_MOTOR_CANBUS = "rio";
  public static final double CTRE_SENSOR_UNITS_PER_ROTATION = 2048;
  
  public static final int RobotNum = 5199;

  // Pneumatics Control Module
  public static final int PCM_ID = 50;

  // Compressor
  public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;

  public static final class Piecemanipulation {
    // Subsystems
    public static final boolean ARM_ELEVATOR_MANUAL = false;

    // Elevator
    public static final boolean ENABLE_ELEVATOR = true;
    public static final int ELEVATOR_MOTOR_ID = 40;
    public static final PID ELEVATOR_PID = new PID(0.05, 0, 0); // 0.08
    
    // Arm
    public static final boolean ENABLE_ARM = true;
    public static final int ARM_EXTEND_MOTOR_ID = 43;
    public static final int ARM_ROTATE_MOTOR_ID = 39;
    public static final PID ARM_ROTATE_PID = new PID(0.03, 0, 0); // 0.08
    public static final PID ARM_EXTEND_PID = new PID(0.05, 0, 0); // 0.05

    // Wrist
    public static final boolean ENABLE_WRIST = true;
    public static final boolean WRIST_MANUAL = false;
    public static final int WRIST_MOTOR_ID = 27;
    public static final PID WRIST_PID = new PID(0.03, 0, 0); // 0.08

    // Intake
    public static final boolean ENABLE_INTAKE = true;
    public static final boolean INTAKE_MANUAL = true;
    public static final int SPIKE_OUT_ID = 11;
    public static final int SPIKE_IN_ID = 8;
    public static final int BottomIntakeMotor_ID = 54;

    // Claw
    public static final boolean ENABLE_CLAW = true;
    public static final int CLAW_IN_ID = 9;
    public static final int CLAW_OUT_ID = 10;
  }

  public static final class Setpoints { 
    // Arm
    public static final double ARM_ROTATE_SETPOINT_HUMANPLAYER = 23;
    public static final double ARM_ROTATE_SETPOINT_STABLE = 0;
    public static final double ARM_ROTATE_SETPOINT_HIGH = -63;
    public static final double ARM_ROTATE_SETPOINT_MID = -60;
    public static final double ARM_ROTATE_SETPOINT_LOW = -75;

    public static final double ARM_EXTEND_SETPOINT_HUMANPLAYER = 11;
    public static final double ARM_EXTEND_SETPOINT_STABLE = 8;
    public static final double ARM_EXTEND_SETPOINT_HIGH = 23;
    public static final double ARM_EXTEND_SETPOINT_MID = 0;
    // public static final double ARM_EXTEND_SETPOINT_LOW = 27;
    
    //Elevator
    public static final double ELEVATOR_SETPOINT_HUMANPLAYER = 34;
    public static final double ELEVATOR_SETPOINT_HIGH = 36;
    public static final double ELEVATOR_SETPOINT_MID = 13;
    public static final double ELEVATOR_SETPOINT_LOW = 0;
  }

  public static final class ControllerIds {
    public static final int FIRST_DRIVER_CONTROLLER = 0;
    public static final int SECOND_DRIVER_CONTROL_STATION = 1;
    public static final int SECOND_DRIVER_CONTROLLER = 2;

    public static final int BUTTON_PANEL_1 = 2;
    public static final int BUTTON_PANEL_2 = 3;

    public static final int XBOX_L_JOY_X = 0;
    public static final int XBOX_L_JOY_Y = 1;

    public static final int XBOX_R_JOY_X = 4;
    public static final int XBOX_R_JOY_Y = 5;

    public static final int XBOX_L_BUMPER = 5;
    public static final int XBOX_R_BUMPER = 6;

    public static final int XBOX_L_TRIGGER = 2;
    public static final int XBOX_R_TRIGGER = 3;

    public static final int XBOX_Y_BUTTON = 4;
    public static final int XBOX_X_BUTTON = 3;
    public static final int XBOX_B_BUTTON = 2;
    public static final int XBOX_A_BUTTON = 1;

    public static final int DRIVER_STATION_TOGGLE_1 = 2;
    public static final int DRIVER_STATION_TOGGLE_2 = 5;
    public static final int DRIVER_STATION_TOGGLE_3 = 6;
    public static final int DRIVER_STATION_TOGGLE_4 = 9;

    public static final int DRIVER_STATION_BUTTON_1 = 1;
    public static final int DRIVER_STATION_BUTTON_2 = 4;
    public static final int DRIVER_STATION_BUTTON_3 = 3;

    public static final int DRIVER_STATION_X_AXIS = 0;
    public static final int DRIVER_STATION_Y_AXIS = 1;
  }

  public static final class FieldConstants {
    public static final double length = Units.feetToMeters(54);
    public static final double width = Units.feetToMeters(27);

    public static final double TOP_CONE_MARKER_TO_EDGE_Z_METERS = 0.98425;
    public static final double TOP_CONE_MARKER_TO_FLOOR_DISTANCE_METERS = 1.0795;
  }

  

}
