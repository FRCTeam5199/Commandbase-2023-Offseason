package frc.robot.configs;

public class Practice extends DefaultConfig{
    public static final int DRIVER_CONTROLLER_PORT = 0;
    //Swerve IDS
    public static final int SWERVE_DRIVE_FL = 0;
    public static final int SWERVE_TURN_FL = 0;
    public static final int SWERVE_DRIVE_FR = 0;
    public static final int SWERVE_TURN_FR = 0;
    public static final int SWERVE_DRIVE_BR = 0;
    public static final int SWERVE_TURN_BR = 0;
    public static final int SWERVE_DRIVE_BL = 0;
    public int SWERVE_TURN_BL = 0;
    //Camera IDS

    //Modes
    public boolean SAFETY_MODE = true;
    public boolean NORMAL = false;

    //Tuning
    double ROTATION_SPEED = 0.001;
}
