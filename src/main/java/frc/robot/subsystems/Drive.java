package frc.robot.subsystems;


import java.io.File;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import swervelib.encoders.CANCoderSwerve;
import swervelib.imu.Pigeon2Swerve;
import swervelib.imu.SwerveIMU;
import swervelib.motors.*;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveModulePhysicalCharacteristics;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import java.util.Objects;



public class Drive extends SubsystemBase {
    XboxController controller;
    TalonFXSwerve fldrivermotor;
    TalonFXSwerve flsteermotor;
    TalonFXSwerve frdrivermotor;
    TalonFXSwerve frsteermotor;
    TalonFXSwerve bldrivermotor;
    TalonFXSwerve blsteermotor;
    TalonFXSwerve brdrivermotor;
    TalonFXSwerve brsteermotor;
    
    CANCoderSwerve fl;
    CANCoderSwerve fr;
    CANCoderSwerve br;
    CANCoderSwerve bl;
    PIDFConfig flDrivePIDF;
    PIDFConfig flSteerPIDF;
    PIDFConfig frDrivePIDF;
    PIDFConfig frSteerPIDF;
    PIDFConfig blDrivePIDF;
    PIDFConfig blSteerPIDF;
    PIDFConfig brDrivePIDF;
    PIDFConfig brSteerPIDF;
    SwerveModulePhysicalCharacteristics frontLeft;
    SwerveModulePhysicalCharacteristics frontRight;
    SwerveModulePhysicalCharacteristics backLeft;
    SwerveModulePhysicalCharacteristics backRight;
    SwerveIMU swerveIMU;
    SwerveModuleConfiguration frontL;
    SwerveModuleConfiguration frontR;
    SwerveModuleConfiguration backL;
    SwerveModuleConfiguration backR;
    SwerveDriveConfiguration swerve;
    SwerveModuleConfiguration swerveConfigs[] = {frontL, frontR, backL, backR};
    SwerveModule frontLModule;
    SwerveModule frontRModule;
    SwerveModule backLModule;
    SwerveModule backRModule;





    SwerveControllerConfiguration swerveConfiguration;

    SwerveController swerveController;

    public SwerveDrive swerver;

    



    double rotation;


    public void init(){
        controller = new XboxController(0);
        setDrivingPID();
        setSteeringPID();
        motorInit();
        encoderInit();
        swerveModuleConfiguration();
        swerveInit();


    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){

        swerver.drive(translation, rotation, false, false);        


        
    }

    public void setDrivingPID(){
        flDrivePIDF = new PIDFConfig(Constants.DriveConstants.PIDF[0], Constants.DriveConstants.PIDF[1], Constants.DriveConstants.PIDF[2], Constants.DriveConstants.PIDF[3]);
        frDrivePIDF = new PIDFConfig(Constants.DriveConstants.PIDF[0], Constants.DriveConstants.PIDF[1], Constants.DriveConstants.PIDF[2], Constants.DriveConstants.PIDF[3]);
        blDrivePIDF = new PIDFConfig(Constants.DriveConstants.PIDF[0], Constants.DriveConstants.PIDF[1], Constants.DriveConstants.PIDF[2], Constants.DriveConstants.PIDF[3]);
        brDrivePIDF = new PIDFConfig(Constants.DriveConstants.PIDF[0], Constants.DriveConstants.PIDF[1], Constants.DriveConstants.PIDF[2], Constants.DriveConstants.PIDF[3]);
        }

    public void setSteeringPID(){
        flSteerPIDF = new PIDFConfig(Constants.DriveConstants.PIDF[0], Constants.DriveConstants.PIDF[1], Constants.DriveConstants.PIDF[2], Constants.DriveConstants.PIDF[3]);
        frSteerPIDF = new PIDFConfig(Constants.DriveConstants.PIDF[0], Constants.DriveConstants.PIDF[1], Constants.DriveConstants.PIDF[2], Constants.DriveConstants.PIDF[3]);
        blSteerPIDF = new PIDFConfig(Constants.DriveConstants.PIDF[0], Constants.DriveConstants.PIDF[1], Constants.DriveConstants.PIDF[2], Constants.DriveConstants.PIDF[3]);
        brSteerPIDF = new PIDFConfig(Constants.DriveConstants.PIDF[0], Constants.DriveConstants.PIDF[1], Constants.DriveConstants.PIDF[2], Constants.DriveConstants.PIDF[3]);
    }

    public void motorInit(){
        fldrivermotor = new TalonFXSwerve(1, "Canivore1", true);
        flsteermotor = new TalonFXSwerve(2, "Canivore1", false);
        frdrivermotor = new TalonFXSwerve(3, "Canivore1", true);
        frsteermotor = new TalonFXSwerve(4, "Canivore1", false);
        bldrivermotor = new TalonFXSwerve(5, "Canivore1", true);
        blsteermotor = new TalonFXSwerve(6, "Canivore1", false);
        brdrivermotor = new TalonFXSwerve(7, "Canivore1", true);
        brsteermotor = new TalonFXSwerve(8, "Canivore1", false);

        fldrivermotor.configurePIDF(flDrivePIDF);
        flsteermotor.configurePIDF(flSteerPIDF);
        frdrivermotor.configurePIDF(frDrivePIDF);
        frsteermotor.configurePIDF(frSteerPIDF);
        bldrivermotor.configurePIDF(blDrivePIDF);
        blsteermotor.configurePIDF(blSteerPIDF);
        brdrivermotor.configurePIDF(brDrivePIDF);
        brsteermotor.configurePIDF(brSteerPIDF);
        
    }

    public void encoderInit(){
        fl = new CANCoderSwerve(Constants.DriveConstants.ENCODERID_FL, "Canivore1");
        fr = new CANCoderSwerve(Constants.DriveConstants.ENCODERID_FR, "Canivore1");
        bl = new CANCoderSwerve(Constants.DriveConstants.ENCODERID_BL, "Canivore1");
        br = new CANCoderSwerve(Constants.DriveConstants.ENCODERID_BR, "Canivore1");
    }

    public void swerveModuleConfiguration(){
        frontLeft = new SwerveModulePhysicalCharacteristics(Constants.DriveConstants.SWERVE_GEAR_RATIO, Constants.DriveConstants.SWERVE_GEAR_RATIO , Constants.DriveConstants.WHEEL_DIAMETER, Constants.DriveConstants.SWERVE_DRIVE_RAMP_RATE, Constants.DriveConstants.SWERVE_STEER_RAMP_RATE, Constants.DriveConstants.ENCODER_PULSE_RATE, Constants.DriveConstants.ENCODER_PULSE_RATE);
        frontRight = new SwerveModulePhysicalCharacteristics(Constants.DriveConstants.SWERVE_GEAR_RATIO, Constants.DriveConstants.SWERVE_GEAR_RATIO , Constants.DriveConstants.WHEEL_DIAMETER, Constants.DriveConstants.SWERVE_DRIVE_RAMP_RATE, Constants.DriveConstants.SWERVE_STEER_RAMP_RATE, Constants.DriveConstants.ENCODER_PULSE_RATE, Constants.DriveConstants.ENCODER_PULSE_RATE);
        backLeft = new SwerveModulePhysicalCharacteristics(Constants.DriveConstants.SWERVE_GEAR_RATIO, Constants.DriveConstants.SWERVE_GEAR_RATIO , Constants.DriveConstants.WHEEL_DIAMETER, Constants.DriveConstants.SWERVE_DRIVE_RAMP_RATE, Constants.DriveConstants.SWERVE_STEER_RAMP_RATE, Constants.DriveConstants.ENCODER_PULSE_RATE, Constants.DriveConstants.ENCODER_PULSE_RATE);
        backRight = new SwerveModulePhysicalCharacteristics(Constants.DriveConstants.SWERVE_GEAR_RATIO, Constants.DriveConstants.SWERVE_GEAR_RATIO , Constants.DriveConstants.WHEEL_DIAMETER, Constants.DriveConstants.SWERVE_DRIVE_RAMP_RATE, Constants.DriveConstants.SWERVE_STEER_RAMP_RATE, Constants.DriveConstants.ENCODER_PULSE_RATE, Constants.DriveConstants.ENCODER_PULSE_RATE);
    
        swerveConfigs[0] = new SwerveModuleConfiguration(fldrivermotor, flsteermotor, fl, Constants.DriveConstants.SWERVE_ANGLE_OFFSET, Constants.DriveConstants.SWERVEFL_POSITION[0],  Constants.DriveConstants.SWERVEFL_POSITION[1], flSteerPIDF, flDrivePIDF, Constants.DriveConstants.MAX_SPEED_RPM, frontLeft, "Front Left Swerve Module");
        swerveConfigs[1] = new SwerveModuleConfiguration(frdrivermotor, frsteermotor, fr, Constants.DriveConstants.SWERVE_ANGLE_OFFSET, Constants.DriveConstants.SWERVEFR_POSITION[0], Constants.DriveConstants.SWERVEFR_POSITION[1], frSteerPIDF, frDrivePIDF, Constants.DriveConstants.MAX_SPEED_RPM, frontRight, "Front Right Swerve Module");
        swerveConfigs[2] = new SwerveModuleConfiguration(bldrivermotor, blsteermotor, bl, Constants.DriveConstants.SWERVE_ANGLE_OFFSET, Constants.DriveConstants.SWERVEBL_POSITION[0], Constants.DriveConstants.SWERVEBL_POSITION[1], blSteerPIDF, blDrivePIDF, Constants.DriveConstants.MAX_SPEED_RPM, backLeft, "Back Left Swerve Module");
        swerveConfigs[3] = new SwerveModuleConfiguration(brdrivermotor, brsteermotor, br, Constants.DriveConstants.SWERVE_ANGLE_OFFSET, Constants.DriveConstants.SWERVEBR_POSITION[0], Constants.DriveConstants.SWERVEBR_POSITION[1], brSteerPIDF, brDrivePIDF, Constants.DriveConstants.MAX_SPEED_RPM, backRight, "Back Right Swerve Module");
    }


    public void swerveInit(){

        swerve = new SwerveDriveConfiguration(swerveConfigs, new Pigeon2Swerve(Constants.DriveConstants.Pigeon_2, "rio"), Constants.DriveConstants.MAX_SPEED_MPS, false);

        swerveConfiguration = new SwerveControllerConfiguration(swerve, new PIDFConfig(Constants.DriveConstants.PIDF[0], Constants.DriveConstants.PIDF[1], Constants.DriveConstants.PIDF[2], Constants.DriveConstants.PIDF[3]));
        
        swerveController = new SwerveController(swerveConfiguration);

        swerver = new SwerveDrive(swerve, swerveConfiguration);

    
    }
    public SwerveController getSwerveController(){
        init();
        return swerveController;
    }

    public Rotation2d getHeading(){
        
      return swerver.getYaw();
    }
}
