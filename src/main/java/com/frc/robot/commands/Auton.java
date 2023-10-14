package com.frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.List;
import java.util.HashMap;
import java.util.List;

import com.frc.robot.misc.Time;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.ExecutionBehavior;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.WaitBehavior;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.frc.robot.subsystems.Drivetrain;
import com.frc.robot.subsystems.piecemanipulation.ClawSubsystem;
import com.frc.robot.subsystems.piecemanipulation.ElevatorSubsystem;
import com.frc.robot.subsystems.piecemanipulation.ArmSubsystem;
import com.frc.robot.subsystems.piecemanipulation.IntakeSubsystem;
import com.frc.robot.subsystems.piecemanipulation.WristSubsystem;
import com.frc.robot.utility.UserInterface;
import com.frc.robot.CompConstants;
import com.frc.robot.subsystems.CompressorSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import static com.frc.robot.utility.UserInterface.AUTON_TAB;

/**
 * 
 * The AutoBuilder class generates commands for autonomous actions using a
 * Drivetrain and path planning.
 * 
 * It uses a HashMap to store "marker" events and corresponding actions, as well
 * as a SwerveAutoBuilder for generating path following commands.
 */
public class Auton {

  /**
   * 
   * A HashMap containing "marker" events and corresponding actions.
   */
  private HashMap<String, Command> eventMap;


  /**
   * 
   * An instance of SwerveAutoBuilder used to generate path following commands.
   */
  private SwerveAutoBuilder autoBuilder;

  private Time timer = new Time();

  /**
   * 
   * An instance of Drivetrain that represents the drive subsystem.
   */
  private Drivetrain drivetrain;
  private SendableChooser<Command> autonChooser;

  private ArmSubsystem arm;
  private IntakeSubsystem intake;
  private ElevatorSubsystem elevator;
  private ClawSubsystem claw;
  private WristSubsystem wrist;

  private SwerveAutoBuilder teleopAutoBuilder;

  /**
   * 
   * Constructor for AutoBuilder class. Initializes eventMap, autoBuilder, and
   * drivetrain.
   * 
   * @param drivetrain An instance of Drivetrain that represents the drive
   *                   ubsystem.
 * @return 
   */
  public Auton(Drivetrain drivetrain, ArmSubsystem arm, IntakeSubsystem intake, ElevatorSubsystem elevator, ClawSubsystem claw, WristSubsystem wrist) {
    this.drivetrain = drivetrain;
    this.arm = arm;
    this.intake = intake;
    this.elevator = elevator;
    this.claw = claw;
    this.wrist = wrist;
    autonChooser = new SendableChooser<>();

    eventMap = new HashMap<>();
    eventMap.put("printCommand", new PrintCommand("test Print command"));
    // eventMap.put("PLACE_CONE_HIGH", new RunCommand( () ->
    // arm.setArmState(ArmPose.HIGH_CONE), arm).until( () ->
    // arm.isArmAtSetpoint()));


    autoBuilder = new SwerveAutoBuilder(
        () -> drivetrain.getOdometryPose2dNoApriltags(), // Pose2d supplier TODO: possibly revert back to no apriltags
        (pose) -> drivetrain.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
        CompConstants.m_kinematics, // SwerveDriveKinematics
        new PIDConstants(1.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        (state) -> drivetrain.autoSetChassisState(state), // Module states consumer used to output to the drive
                                                          // subsystem
        eventMap,
        true,
        drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
    );

    teleopAutoBuilder = new SwerveAutoBuilder(
        () -> drivetrain.getPose(), // Pose2d supplier
        (pose) -> drivetrain.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
        CompConstants.m_kinematics, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.6, 0.1, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        (state) -> drivetrain.autoSetChassisState(state), // Module states consumer used to output to the drive
                                                          // subsystem
        eventMap,
        false,
        drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
    );
    
    // List of auton options, must be manually edited
    //autonChooser.setDefaultOption("Taxi Wall", TaxiWall());
    //autonChooser.addOption("Nothing", nothingCommand());
    //autonChooser.addOption("Taxi and Level", TaxiandLevel());
    //autonChooser.addOption("Taxi Cube Level", RedTaxiCubeLevel());
    //autonChooser.addOption("Taxi Cube Level 180", RedTaxiCubeLevel180());
    //autonChooser.addOption("Taxi Wall Cube", TaxiWallCube());
    //autonChooser.addOption("Taxi Wall Cube Shoot", TaxiWallCubeShoot());
    autonChooser.setDefaultOption("Nothing", doNothing());
    autonChooser.setDefaultOption("Taxi and Level", TaxiandLevel());
    autonChooser.addOption("Correct Blue Taxi Cube Level", CBlueTaxiCubeLevel());
    autonChooser.addOption("Correct Red Taxi Cube Level", CRedTaxiCubeLevel());
    autonChooser.addOption("Correct Blue Taxi Cube Level 180", CBlueTaxiCubeLevel180());
    autonChooser.addOption("Correct Red Taxi Cube Level 180", CRedTaxiCubeLevel180());
    autonChooser.addOption("Correct Blue Taxi Wall", CBlueTaxiWall());
    autonChooser.addOption("Correct Red Taxi Wall", CRedTaxiWall());
    autonChooser.addOption("Correct Blue Taxi Wall Cube", CBlueTaxiWallCube());
    autonChooser.addOption("Correct Red Taxi Wall Cube", CRedTaxiWallCube());
    autonChooser.addOption("Correct Blue Taxi HP Cube", CBlueTaxiHPCube());
    autonChooser.addOption("Correct Red Taxi HP Cube", CRedTaxiHPCube());
    autonChooser.addOption("Correct Red Taxi HP 3 piece", CRedTaxiHPCubeCube());
    autonChooser.addOption("Correct Red Taxi HP", CRedTaxiHP());
    autonChooser.addOption("Correct Blue Taxi HP", CBlueTaxiHP());
    // autonChooser.addOption("Correct Red taxi level", CRedTaxiLevel());
    // autonChooser.addOption("Correct Blue taxi level", CBlueTaxiLevel());
    autonChooser.addOption("Level", Level());
    //autonChooser.addOption("AutonTest", Apriltagtest());

    Shuffleboard.getTab("Auton").add("Auton Style", autonChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1);;
    //autonChooser.addOption("Red Taxi Cube Level", RedTaxiCubeLevel());

    Shuffleboard.getTab("Auton").add(autonChooser);
}

  public Command getAuton() {
    return autonChooser.getSelected();
  }

  public Command doNothing(){
    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(0.3), intake.retractPiston(), new WaitCommand(14));
  }

  public Command getAutoCommand(String pathName) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(1.5, 0.75));

    return autoBuilder.fullAuto(pathGroup);
  }

  public Command Apriltagtest(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("ungadunga", 2,2,true);

    return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1));
  }

  public Command CBlueTaxiCubeLevel(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Blue Taxi Cube Level p1", 2, 2);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Blue Taxi Cube Level p2", 2,2);

    return new SequentialCommandGroup(intake.deployPiston(),new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), intake.deployPiston(), intake.dropAndStop().alongWith(autoBuilder.fullAuto(pathGroup2)), new ChargingStationAuto(drivetrain));
  }

  public Command CRedTaxiCubeLevel(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Blue Taxi Cube Level p1", 2, 2, true);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Blue Taxi Cube Level p2", 2,2, true);

    return new SequentialCommandGroup(intake.deployPiston(),new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), intake.deployPiston(), intake.dropAndStop().alongWith(autoBuilder.fullAuto(pathGroup2)), new ChargingStationAuto(drivetrain));

  }

  public Command CBlueTaxiCubeLevel180(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Blue Taxi Cube Level p1", 2, 2);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Blue Taxi Cube Level p2", 2,2);
    List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("180", new PathConstraints(1, 1));

    return new SequentialCommandGroup(intake.deployPiston(),new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), intake.deployPiston(), intake.dropAndStop().alongWith(autoBuilder.fullAuto(pathGroup2)), intake.slowIntake(), intake.retractPiston(), new ChargingStationAuto180(drivetrain), autoBuilder.fullAuto(pathGroup3), intake.fastOutake().alongWith(new ChargingStationAuto(drivetrain)));
  }

  public Command CBlueTaxiWall(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Correct Blue Taxi Wall", 2, 2);

    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1));
  }

  public Command CRedTaxiCubeLevel180(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Blue Taxi Cube Level p1", 2, 2, true);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Blue Taxi Cube Level p2", 2,2, true);
    List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("180", new PathConstraints(1, 1));

    return new SequentialCommandGroup(intake.deployPiston(),new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), intake.deployPiston(), intake.dropAndStop().alongWith(autoBuilder.fullAuto(pathGroup2)), intake.slowIntake(), intake.retractPiston(), new ChargingStationAuto180(drivetrain), autoBuilder.fullAuto(pathGroup3), intake.fastOutake().alongWith(new ChargingStationAuto(drivetrain)));

  }

  public Command CRedTaxiWall(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Correct Blue Taxi HP", 2, 2, true);

    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1));
  }

  public Command CBlueTaxiHP(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Correct Blue Taxi HP", 4, 2);
    return new SequentialCommandGroup(intake.deployPiston(),new WaitCommand(.2), autoBuilder.fullAuto(pathGroup1));
  }
  public Command CRedTaxiHP(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Correct Red Taxi HP", 4, 2, true);
    return new SequentialCommandGroup(intake.deployPiston(),new WaitCommand(.2), autoBuilder.fullAuto(pathGroup1));
  }
  public Command CBlueTaxiHPCube(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Correct Blue Taxi HP", 4, 2);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Correct Red Taxi HP Back", 4, 3);
    return new SequentialCommandGroup(intake.deployPiston(),new WaitCommand(.2), autoBuilder.fullAuto(pathGroup1).alongWith(intake.intake()),intake.retractPiston(), intake.stopSpin(),autoBuilder.fullAuto(pathGroup2), intake.outtake(),new WaitCommand(.3),intake.stopSpin());
  }

  public Command CRedTaxiHPCube(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Correct Red Taxi HP", 4, 3, true);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Correct Red Taxi HP Back", 3, 2,true);
    return new SequentialCommandGroup(intake.deployPiston(),new WaitCommand(.2), autoBuilder.fullAuto(pathGroup1).alongWith(intake.intake()),intake.retractPiston(), intake.stopSpin(),autoBuilder.fullAuto(pathGroup2), intake.outtake(),new WaitCommand(.3),  intake.stopSpin());
  }

  public Command CRedTaxiHPCubeCube(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Correct Red Taxi HP", 4, 3, true);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Correct Red Taxi HP Back", 5, 3,true);
    List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("Correct Blue HP 3 piece pt1",8,4,true);
    List<PathPlannerTrajectory> pathGroup4 = PathPlanner.loadPathGroup("Correct Blue HP 3 piece pt2",7,4,true);
    return new SequentialCommandGroup(intake.deployPiston(),new WaitCommand(.2), autoBuilder.fullAuto(pathGroup1).alongWith(intake.intake()),intake.retractPiston(), intake.stopSpin(),autoBuilder.fullAuto(pathGroup2), intake.outtake(), new WaitCommand(.5), intake.outtake(), autoBuilder.fullAuto(pathGroup3).alongWith(new WaitCommand(1.5).andThen(intake.deployPiston().andThen(intake.intake()))),intake.retractPiston(),autoBuilder.fullAuto(pathGroup4), intake.outtake());
  }

  public Command CRedTaxiWallCube(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Correct Blue Taxi HP", 2, 2, true);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Correct Red Taxi Back Wall", 2, 2, true);

    //return new SequentialCommandGroup(intake.deployPiston(), intake.intake().alongWith(autoBuilder.fullAuto(pathGroup1)), intake.slowIntake(), intake.retractPiston(), autoBuilder.fullAuto(pathGroup2), intake.outtake(), new WaitCommand(.5), intake.stopSpin());
    return new SequentialCommandGroup(intake.deployPiston(), intake.intake().alongWith(autoBuilder.fullAuto(pathGroup1)), intake.intake(), intake.retractPiston(), autoBuilder.fullAuto(pathGroup2), intake.outtake(), new WaitCommand(.5), intake.stopSpin());
  }

  public Command CBlueTaxiWallCube(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Correct Blue Taxi Wall", 2, 2);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Correct Blue Taxi Back Wall", 2, 2);

    //return new SequentialCommandGroup(intake.deployPiston(), intake.intake().alongWith(autoBuilder.fullAuto(pathGroup1)), intake.slowIntake(), intake.retractPiston(), autoBuilder.fullAuto(pathGroup2), intake.outtake(), new WaitCommand(.5), intake.stopSpin());
    return new SequentialCommandGroup(intake.deployPiston(), intake.intake().alongWith(autoBuilder.fullAuto(pathGroup1)), intake.intake(), intake.retractPiston(), autoBuilder.fullAuto(pathGroup2), intake.outtake(), new WaitCommand(.5), intake.stopSpin());
  }


  public Command Level(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("SOMETHING", 2,2);

    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), new ChargingStationAuto(drivetrain));
  } 

  public Command 
  TaxiandLevel(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Taxi and Level Blue", 1.4,2); //1.4

    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.5), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), new ChargingStationAuto(drivetrain));
  }

  // public Command TaxiandLevel(){
  //   List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Taxi and Level Red", new PathConstraints(1.5, 2));

  //   return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2 ), intake.retractPiston(), autoBuilder.fullAuto(pathGroup.get(0)), new ChargingStationAuto(drivetrain));
  // }

  /*public Command RedTaxiCubeLevel(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("TaxiOverCharge Red", new PathConstraints(2, 2));
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("ChargeFromPiece", new PathConstraints(1.5, 2));
    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), intake.deployPiston(), intake.dropAndStop().alongWith(autoBuilder.fullAuto(pathGroup2)), new ChargingStationAuto(drivetrain));
  }*/

  //removed

  // public Command BlueTaxiCubeLevel(){
  //   List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("TaxiOverCharge Red", 2, 2, true);
  //   List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("ChargeFromPiece Red", 2,2,true);

  //   return new SequentialCommandGroup(intake.deployPiston(),new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), intake.deployPiston(), intake.dropAndStop().alongWith(autoBuilder.fullAuto(pathGroup2)), new ChargingStationAuto(drivetrain));

  // }
  
  // public Command BlueTaxiCubeLevel180(){
  //   List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("TaxiOverCharge Red", 2, 2, true);
  //   List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("ChargeFromPiece Red", 1.7, 2, true);
  //   List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("180", new PathConstraints(1, 1));


  //   return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), intake.deployPiston(), intake.dropAndStop().alongWith(autoBuilder.fullAuto(pathGroup2)), intake.intake(), intake.retractPiston(), new ChargingStationAuto180(drivetrain), autoBuilder.fullAuto(pathGroup3),intake.fastOutake().alongWith(new ChargingStationAuto(drivetrain)));
  // }

  // public Command RedTaxiCubeLevel180(){
  //   List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("TaxiOverCharge", new PathConstraints(2, 2));
  //   List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("ChargeFromPiece", new PathConstraints(1.7, 2));
  //   List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("180", new PathConstraints(1, 1));

  //   double initHeading = drivetrain.getHeading();


  //   return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), new ZeroHeading(drivetrain), intake.deployPiston(), intake.dropAndStop().alongWith(autoBuilder.fullAuto(pathGroup2)), intake.retractPiston(), new ChargingStationAuto180(drivetrain), autoBuilder.fullAuto(pathGroup3),intake.autonOuttake().alongWith(new ChargingStationAuto(drivetrain)));
  // }

  // public Command nothingCommand(){
  //   return null;
  // }
  
  // // public Command TaxiCubeShootWall(){
  // //   List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall", new PathConstraints(1.25, 0.9));
  // //   List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Red Taxi Wall Back", new PathConstraints(1.25, 1));
    
  // //   return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1.get(0)), autoBuilder.fullAuto(pathGroup2.get(0)));
  // // }
  // /**
  //  * only goes foward and cube shooter
  //  * @return
  //  */
  // public Command TaxiWall() {
  //   List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall", new PathConstraints(1.25, 0.9));
  //   return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(1),intake.retractPiston(), autoBuilder.fullAuto(pathGroup1.get(0)));
  // }
  
  // /**
  //  * 
  //  * @return only moves foward and backward on side of wall cube shooter
  //  */
  // public Command TaxiWallCube(){
  //   List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall", new PathConstraints(1.25, 0.9));
  //   List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Red Taxi Wall Back", new PathConstraints(1.25, 1));
    
  //   return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(1), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1.get(0)), autoBuilder.fullAuto(pathGroup2.get(0)));
  // }
  // /**
  //  * 
  //  * @return moves foward and backwards cube shooter and picks up cube and scores
  //  */
  // public Command TaxiWallCubeShoot(){
  //   List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall", new PathConstraints(1.25, 0.9));
  //   List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Red Taxi Wall Back Turn End", new PathConstraints(1.25, 1));
    
  //   return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.25), autoBuilder.fullAuto(pathGroup1.get(0)).alongWith(intake.intake()), intake.retractPiston(), autoBuilder.fullAuto(pathGroup2.get(0)), intake.outtake(), new WaitCommand(.50), intake.stopSpin());
  // }
  


  // public Command getTaxiCharge(){
  //   List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("TaxiCharge", new PathConstraints(2, 1));

  //   return new SequentialCommandGroup(
  //     autoBuilder.fullAuto(pathGroup.get(0)),
  //     new ChargingStationAuto(drivetrain)
  //   );
  // }



  // public Command getCharge(){
  //   List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Charge", new PathConstraints(1, 1));

  //   return new SequentialCommandGroup(
  //     autoBuilder.fullAuto(pathGroup.get(0)),
  //     new ChargingStationAuto(drivetrain)
  //   );
  // }
  
  // public Command BlueTaxiWallCubeShoot() {
  //   List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Blue Taxi Wall", 2, 1, true);
  //   List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Red Taxi Wall Back Turn End", 2, 1.5, true);

  //   return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.25), autoBuilder.fullAuto(pathGroup1.get(0)).alongWith(intake.intake()), intake.retractPiston(), autoBuilder.fullAuto(pathGroup2.get(0)), intake.outtake(), new WaitCommand(.50), intake.stopSpin());
  // }

  // public Command RedTaxiWallCubeShootCube(){
  //   List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall", new PathConstraints(3,3 ));
  //   List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Red Taxi Wall Back Turn End dif", new PathConstraints(2, 1.5));
  //   List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("Red Taxi Wall Back Turn Cube", new PathConstraints(2, 1.5));

  //   return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.25), autoBuilder.fullAuto(pathGroup1.get(0)).alongWith(intake.intake()), 
  //   intake.retractPiston(), autoBuilder.fullAuto(pathGroup2.get(0)), intake.outtake(), new WaitCommand(.50), intake.stopSpin());
  // }
  // public Command RedTaxiWallCubeShootCubeShoot(){
  //   List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall", 4,4, false);
  //   List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Red Taxi Wall Back Turn End dif", 3, 1.5, false);
  //   List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("Red Taxi Wall Back Turn cube", 3, 1.5, false);
  //   List<PathPlannerTrajectory> pathGroup4 = PathPlanner.loadPathGroup("Red Taxi Wall Back Turn cube end", 3, 1.5, false);

  //   return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1.get(0)),autoBuilder.fullAuto(pathGroup2.get(0)),autoBuilder.fullAuto(pathGroup3.get(0)), autoBuilder.fullAuto(pathGroup4.get(0)));
  // }
  // public Command RedTaxiHP(){
  //   List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi HP",3, 2, false);
  //   return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1.get(0)));
  // }
  // public Command RedTaxiHPCube(){
  //   List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi HP",3, 2, false);
  //   List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Red Taxi HP Back",3, 2, false);

  //   return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(0.25), autoBuilder.fullAuto(pathGroup1.get(0)).alongWith(intake.intake()),intake.retractPiston(), autoBuilder.fullAuto(pathGroup2.get(0)), intake.fastOutake(), new WaitCommand(0.25), intake.stopSpin());
  // }
  // public Command BlueTaxiWall(){
  //   List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Blue Taxi Wall",3, 1.5, true);
  //   return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1.get(0)));
  // }
  // public Command BlueTaxiHP(){
  //   List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Blue Taxi HP",new PathConstraints(1.25, 0.9));
  //   return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1.get(0)));
  // }  
  // public Command BlueTaxiHPCubeShootFake(){
  //   List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall",3, 3, true);
  //   List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Blue Taxi Wall Back Turn End dif",2, 1.5, true);
  //   return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1.get(0)), autoBuilder.fullAuto(pathGroup2.get(0)));
  // }






  
  public Command goToPose(Pose2d desiredPose, Rotation2d heading) {

    Pose2d currPose = drivetrain.getPose();

    ChassisSpeeds currChassisSpeeds = drivetrain.getChassisSpeeds();

    double currSpeed = Math.abs(Math.hypot(currChassisSpeeds.vxMetersPerSecond, currChassisSpeeds.vyMetersPerSecond));

    PathPlannerTrajectory traj = PathPlanner.generatePath(
        new PathConstraints(2.0, 1.0),
        // position, heading(direction of travel), holonomic rotation, velocity verride
        new PathPoint(currPose.getTranslation(), heading, currPose.getRotation(), currSpeed),
        new PathPoint(desiredPose.getTranslation(), heading, desiredPose.getRotation()));

    return teleopAutoBuilder.followPath(traj);
  }

}