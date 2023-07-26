package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Translation2d;
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
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;



public class Drive extends SubsystemBase {
    CommandXboxController controller;
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

    SwerveDrive swerver;

    



    double rotation;

    public void init(){
        controller = new CommandXboxController(0);
        setDrivingPID();
        setSteeringPID();
        motorInit();
        encoderInit();
        swerveModuleConfiguration();
        swerveInit();


    }

    public void drive(){
        rotation = controller.getRightX() * Constants.DriveConstants.ROTATION_SPEED;
  
        swerver.drive(new Translation2d(controller.getLeftX(), controller.getLeftY()), rotation, false, false);

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

    public boolean isTurning(){
        return controller.getRightX() > 0 || controller.getRightX() < 0;
    }

    public void motorInit(){
        fldrivermotor = new TalonFXSwerve(1, true);
        flsteermotor = new TalonFXSwerve(2, false);
        frdrivermotor = new TalonFXSwerve(3, true);
        frsteermotor = new TalonFXSwerve(4, false);
        bldrivermotor = new TalonFXSwerve(7, true);
        blsteermotor = new TalonFXSwerve(8, false);
        brdrivermotor = new TalonFXSwerve(5, true);
        brsteermotor = new TalonFXSwerve(6, false);

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

    public void Swerver(){
        frontLModule = new SwerveModule(1, frontL);
        frontRModule = new SwerveModule(2, frontR);
        backLModule = new SwerveModule(3, backL);
        backRModule = new SwerveModule(4, backR);
    }

    public void swerveModuleConfiguration(){
        frontLeft = new SwerveModulePhysicalCharacteristics(Constants.DriveConstants.SWERVE_GEAR_RATIO, Constants.DriveConstants.SWERVE_GEAR_RATIO , Constants.DriveConstants.WHEEL_DIAMETER, 1, 1, 2048, 2048);
        frontRight = new SwerveModulePhysicalCharacteristics(Constants.DriveConstants.SWERVE_GEAR_RATIO, Constants.DriveConstants.SWERVE_GEAR_RATIO , Constants.DriveConstants.WHEEL_DIAMETER, 1, 1, 2048, 2048);
        backLeft = new SwerveModulePhysicalCharacteristics(Constants.DriveConstants.SWERVE_GEAR_RATIO, Constants.DriveConstants.SWERVE_GEAR_RATIO , Constants.DriveConstants.WHEEL_DIAMETER, 1, 1, 2048, 2048);
        backRight = new SwerveModulePhysicalCharacteristics(Constants.DriveConstants.SWERVE_GEAR_RATIO, Constants.DriveConstants.SWERVE_GEAR_RATIO , Constants.DriveConstants.WHEEL_DIAMETER, 1, 1, 2048, 2048);

        swerveConfigs[0] = new SwerveModuleConfiguration(fldrivermotor, flsteermotor, fl, Constants.DriveConstants.SWERVE_ANGLE_OFFSET, Constants.DriveConstants.SWERVEFL_POSITION[0],  Constants.DriveConstants.SWERVEFL_POSITION[1], flSteerPIDF, flDrivePIDF, 6380.0, frontLeft, "Front Left Swerve Module");
        swerveConfigs[1] = new SwerveModuleConfiguration(frdrivermotor, frsteermotor, fr, Constants.DriveConstants.SWERVE_ANGLE_OFFSET, Constants.DriveConstants.SWERVEFR_POSITION[0], Constants.DriveConstants.SWERVEFR_POSITION[1], frSteerPIDF, frDrivePIDF, 6380, frontRight, "Front Right Swerve Module");
        swerveConfigs[2] = new SwerveModuleConfiguration(bldrivermotor, blsteermotor, bl, Constants.DriveConstants.SWERVE_ANGLE_OFFSET, Constants.DriveConstants.SWERVEBL_POSITION[0], Constants.DriveConstants.SWERVEBL_POSITION[1], blSteerPIDF, blDrivePIDF, 6380, backLeft, "Back Left Swerve Module");
        swerveConfigs[3] = new SwerveModuleConfiguration(brdrivermotor, brsteermotor, br, Constants.DriveConstants.SWERVE_ANGLE_OFFSET, Constants.DriveConstants.SWERVEBR_POSITION[0], Constants.DriveConstants.SWERVEBR_POSITION[1], brSteerPIDF, brDrivePIDF, 6380, backRight, "Back Right Swerve Module");

        


    }


    public void swerveInit(){


        swerve = new SwerveDriveConfiguration(swerveConfigs, new Pigeon2Swerve(Constants.DriveConstants.Pigeon_2, "rio"), 6380, false);
        
        swerveConfiguration = new SwerveControllerConfiguration(swerve, new PIDFConfig(Constants.DriveConstants.PIDF[0], Constants.DriveConstants.PIDF[1], Constants.DriveConstants.PIDF[2], Constants.DriveConstants.PIDF[3]));
        
        swerveController = new SwerveController(swerveConfiguration);

        swerver = new SwerveDrive(swerve, swerveConfiguration);



    
    }
}
