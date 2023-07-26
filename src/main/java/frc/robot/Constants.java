// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;


  }

  public static class DriveConstants {
    public static final double ROTATION_SPEED = 0.1;

    //Swerve Encoder Ids
    public static final int ENCODERID_FL = 11;
    public static final int ENCODERID_FR = 12;
    public static final int ENCODERID_BL = 14;
    public static final int ENCODERID_BR = 13;

    //SwerveModule individual motor ids
    public static final int SWERVEFL_DRIVER = 1;
    public static final int SWERVEFL_STEER = 2;
    public static final int SWERVEFR_DRIVER = 3;
    public static final int SWERVEFR_STEER = 4;
    public static final int SWERVEBL_DRIVER = 7;
    public static final int SWERVEBL_STEER = 8;
    public static final int SWERVEBR_DRIVER = 5;
    public static final int SWERVEBR_STEER = 6;

    //SwerveModule characteristics
    public static final double SWERVE_GEAR_RATIO = 6.12;
    public static final double SWERVE_ANGLE_OFFSET = 0;////random number for now
    public static final double WHEEL_DIAMETER = 0.1016;//meters
    public static final double[] SWERVEFL_POSITION = {2.0, 3.0};//random number for now
    public static final double[] SWERVEFR_POSITION = {-2.0, 3.0};//random number for now
    public static final double[] SWERVEBL_POSITION = {2.0, -3.0};//random number for now
    public static final double[] SWERVEBR_POSITION = {-2.0, -3.0};//random number for now
    public static final double MAX_SPEED = 6380;//Rotation per min
    public static final double SWERVE_RAMP_RATE = 1;//Time it takes to go from 0 to max speed in seconds.
    public static final double[] PIDF = {.514, .0514, 0, 5.14};

    //Encoder Characteristics
    public static final double ENCODER_PULSE_RATE = 2048;//hertz


    //Pigeon2 ID
    public static final int Pigeon_2 = 22;//fix



    



    




  }
}
