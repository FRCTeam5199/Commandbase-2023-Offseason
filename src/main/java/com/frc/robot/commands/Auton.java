package com.frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.nio.file.Path;
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
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.server.PathPlannerServer;
import com.frc.robot.subsystems.Drivetrain;
import com.frc.robot.subsystems.piecemanipulation.ClawSubsystem;
import com.frc.robot.subsystems.piecemanipulation.ElevatorSubsystem;
import com.frc.robot.subsystems.piecemanipulation.ArmSubsystem;
import com.frc.robot.subsystems.piecemanipulation.IntakeSubsystem;
import com.frc.robot.subsystems.piecemanipulation.WristSubsystem;
import com.frc.robot.Constants;
import com.frc.robot.subsystems.CompressorSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.shuffleboard.*;

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
  private SwerveAutoBuilder autoBuilder2;
  private SwerveAutoBuilder autoBuilder3;


  private Time timer = new Time();

  /**
   * 
   * An instance of Drivetrain that represents the drive subsystem.
   */
  private Drivetrain drivetrain;

  private ArmSubsystem arm;
  private IntakeSubsystem intake;
  private ElevatorSubsystem elevator;
  private ClawSubsystem claw;
  private WristSubsystem wrist;
  public SendableChooser<Command> autonChooser = new SendableChooser<>();

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

    eventMap = new HashMap<>();


    
    // eventMap.put("PLACE_CONE_HIGH", new RunCommand( () ->
    // arm.setArmState(ArmPose.HIGH_CONE), arm).until( () ->
    // arm.isArmAtSetpoint()));


    autoBuilder = new SwerveAutoBuilder(
        () -> drivetrain.getOdometryPose2dAprilTags(), // Pose2d supplier TODO: possibly revert back to no apriltags
        (pose) -> drivetrain.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.m_kinematics, // SwerveDriveKinematics
        new PIDConstants(1.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.8, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        (state) -> drivetrain.autoSetChassisState(state), // Module states consumer used to output to the drive
                                                          // subsystem
        eventMap,
        true,
        drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
    );
    
    teleopAutoBuilder = new SwerveAutoBuilder(
      () -> drivetrain.getOdometryPose2dAprilTags(), // Pose2d supplier TODO: possibly revert back to no apriltags
      (pose) -> drivetrain.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
      Constants.m_kinematics, // SwerveDriveKinematics
      new PIDConstants(1.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(0.8, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      (state) -> drivetrain.autoSetChassisState(state), // Module states consumer used to output to the drive
                                                        // subsystem
      eventMap,
      false,
      drivetrain 
    );
    autoBuilder3 = new SwerveAutoBuilder(() -> drivetrain.getOdometryPose2dAprilTags(), // Pose2d supplier TODO: possibly revert back to no apriltags
    (pose) -> drivetrain.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
    Constants.m_kinematics, // SwerveDriveKinematics
    new PIDConstants(1.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0.8, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    (state) -> drivetrain.autoSetChassisState(state), // Module states consumer used to output to the drive
                                                      // subsystem
    eventMap,
    false,
    drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
);

    autoBuilder3 = new SwerveAutoBuilder(()-> drivetrain.getOdometryPose2dAprilTags(), (pose) -> drivetrain.resetOdometry(pose), Constants.m_kinematics, new PIDConstants(.1,0,0), new PIDConstants(1,0,0),(states)->drivetrain.autoSetChassisState(states), eventMap,false,drivetrain);

    //Autons must be manually entered into the auton chooser
    autonChooser.setDefaultOption("Nothing", doNothing());
    autonChooser.setDefaultOption("Taxi and Level", TaxiandLevel());
    autonChooser.addOption("Correct Blue Taxi Cube Level", CBlueTaxiCubeLevel());
    autonChooser.addOption("Correct Red Taxi Cube Level", CRedTaxiCubeLevel());
    autonChooser.addOption("Correct Blue Taxi Wall", CBlueTaxiWall());
    autonChooser.addOption("Correct Red Taxi Wall", CRedTaxiWall());

    autonChooser.addOption("Level", Level());

    autonChooser.addOption("RedHP3Piece", RedHP3Piece());
    autonChooser.addOption("RedHP2Piece", RedHP2Piece());

    autonChooser.addOption("Red 3 Piece and Level", Red3PieceandLevel());
    autonChooser.addOption("Blue 3 Piece and Level", Blue3PieceandLevel());
    autonChooser.addOption("Red 4 Piece HP", RedHP4Piece());
    autonChooser.addOption("Red 3 Piece HP", RedHP3Piece());
    autonChooser.addOption("Red 2 Piece HP charge", RedHP2PieceCharge());
    autonChooser.addOption("Red 2 Piece HP", RedHP2Piece());
    autonChooser.addOption("Blue 3 Piece HP", BlueHP3Piece());
    autonChooser.addOption("Blue 2 Piece HP", BlueHP2Piece());
    autonChooser.addOption("Red 2 Piece Bump Side", TwoPieceRedB());
    autonChooser.addOption("Blue 2 Piece Bump Side", TwoPieceBlueB());
    autonChooser.addOption("Red 3 Piece Bump Side", ThreePieceRedB());
    autonChooser.addOption("Blue 3 Piece Bump Side", ThreePieceBlueB());
    autonChooser.addOption("3 Piece Level Red", ThreePieceLevelRed());
    autonChooser.addOption("Left 2 Piece Center", LeftTwoPieceCenterLevel());
    autonChooser.addOption("Right 2 Piece Center", RightTwoPieceCenterLevel());



    autonChooser.addOption("test", test());

    //Wrong Autons
    // autonChooser.addOption("Red Taxi Cube Level", RedTaxiCubeLevel());
    // autonChooser.addOption("Blue Taxi Cube Level", BlueTaxiCubeLevel());
    // autonChooser.addOption("Red Taxi Wall", RedTaxiWall());
    // autonChooser.addOption("Blue Taxi Wall", BlueTaxiWall());
    // autonChooser.addOption("Red Taxi Wall Cube", RedTaxiWallCube());
    // autonChooser.addOption("Red Taxi Wall Cube Shoot", RedTaxiWallCubeShoot());
    // autonChooser.addOption("Blue Taxi Wall Cube Shoot", BlueTaxiWallCubeShoot());
    // autonChooser.addOption("Red Taxi Cube Level 180", RedTaxiCubeLevel180());
    // autonChooser.addOption("Blue Taxi Cube Level 180", BlueTaxiCubeLevel180());
    // autonChooser.addOption("Red Taxi Over Charge", RedTaxiOverCharge());
    // autonChooser.addOption("Blue Taxi Over Charge", BlueTaxiOverCharge());
    // autonChooser.addOption("Red Taxi Over Charge Level", RedTaxiOverChargeLevel());
    // autonChooser.addOption("Blue Taxi Over Charge Level", BlueTaxiOverChargeLevel());
    // autonChooser.addOption("Blue Taxi HP Cube Shoot", BlueTaxiHPCubeShoot());
    // autonChooser.addOption("Red Taxi Wall Cube Shoot Cube", RedTaxiWallCubeShootCube());
    // autonChooser.addOption("Red Taxi Wall Cube Shoot Cube Shoot", RedTaxiWallCubeShootCubeShoot());
    // autonChooser.addOption("Red Taxi HP", RedTaxiHP());
    // autonChooser.addOption("Red Taxi HP Cube", RedTaxiHPCube());
    eventMap.put("up", intake.retractPiston());
    eventMap.put("down", intake.deployPiston());
    eventMap.put("spin", intake.intake());
    eventMap.put("stop spin", intake.stopSpin());
    eventMap.put("outtake", intake.outtake());
    eventMap.put("slow outtake", intake.slowOutake());

    Shuffleboard.getTab("Auton").add("Auton Style",autonChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1);;

  }

  public Command getAuton(){
    return autonChooser.getSelected();

  }

  public Command doNothing(){
    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(0.3), intake.retractPiston(), new WaitCommand(14));
  }
  
  public Command getAutoCommand(String pathName) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(1.5, 0.75));

    return autoBuilder.fullAuto(pathGroup);
  }

  /*
  public Command CBlueTaxiLevel(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Taxi and Level Blue", 1,1);
    return new SequentialCommandGroup(intake.deployPiston(),new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1));
  }
  public Command CRedTaxiLevel(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Taxi and Level Blue", 1,1, true);
    return new SequentialCommandGroup(intake.deployPiston(),new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1));
  }
  */

  public Command Apriltagtest(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("ungadunga", 2,2,true);

    return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1));
  }

  public Command LeftTwoPieceCenterLevel(){
    PathPlannerTrajectory path = PathPlanner.loadPath("Left 2 Piece Center Red", 2,2);

    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2), autoBuilder3.fullAuto(path));
  }

  public Command RightTwoPieceCenterLevel(){
    PathPlannerTrajectory path = PathPlanner.loadPath("Left 2 Piece Center Red", 2,2, true);

    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2), autoBuilder3.fullAuto(path));
  }

  public Command TwoPieceRedB(){
    PathPlannerTrajectory path = PathPlanner.loadPath("2Piece Bump Red", 2, 2, false);

    return new SequentialCommandGroup(autoBuilder3.fullAuto(path));
  }

  public Command ThreePieceRedB(){
    PathPlannerTrajectory path = PathPlanner.loadPath("3Piece Bump Red", 3, 2);

    return new SequentialCommandGroup(autoBuilder3.fullAuto(path));
  }

  public Command TwoPieceBlueB(){
    PathPlannerTrajectory path = PathPlanner.loadPath("2Piece Bump Blue", 2,2);

    return new SequentialCommandGroup(autoBuilder3.fullAuto(path));
  }

  public Command ThreePieceBlueB(){
    PathPlannerTrajectory path = PathPlanner.loadPath("3Piece Bump Blue", 2,2);

    return new SequentialCommandGroup(autoBuilder3.fullAuto(path));

  }

  public Command ThreePieceLevelRed(){
    PathPlannerTrajectory path = PathPlanner.loadPath("3 Piece Level Red", 2,2);

    return new SequentialCommandGroup(autoBuilder3.fullAuto(path), new ChargingStationAuto(drivetrain));
  }


  public Command Blue3PieceandLevel(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("2.5 Piece and Level p1", 4, 2);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("2.5 Piece and Level p2", 4, 2);
    List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("2.5 Piece and Level p3", 4, 2);
    List<PathPlannerTrajectory> pathGroup4 = PathPlanner.loadPathGroup("2.5 Piece and Level p4", 4, 2);

    return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1).alongWith(new WaitCommand(3).andThen(intake.deployPiston().andThen(intake.intake()))), autoBuilder.fullAuto(pathGroup2).alongWith(intake.retractPiston()), intake.outtake(), autoBuilder.fullAuto(pathGroup3).alongWith(new WaitCommand(2).andThen(intake.deployPiston().andThen(intake.intake()))), autoBuilder.fullAuto(pathGroup4).alongWith(intake.retractPiston()), intake.outtake(), new ChargingStationAuto(drivetrain));


  }

  public Command Red3PieceandLevel(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("2.5 Piece and Level p1", 4, 2);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("2.5 Piece and Level p2", 4, 2);
    List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("2.5 Piece and Level p3", 4, 2);
    List<PathPlannerTrajectory> pathGroup4 = PathPlanner.loadPathGroup("2.5 Piece and Level p4", 4, 2);

    return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1), intake.deployPiston(), autoBuilder.fullAuto(pathGroup2).alongWith(intake.intake()), intake.retractPiston(), new WaitCommand(.3), intake.outtake(), autoBuilder.fullAuto(pathGroup3).alongWith(new WaitCommand(2)).andThen(intake.deployPiston()), intake.intake(), autoBuilder.fullAuto(pathGroup4).alongWith(intake.intake()), intake.retractPiston(), new WaitCommand(.3), intake.outtake(), new ChargingStationAuto(drivetrain));
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

  public Command CRedTaxiCubeLevel180(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Blue Taxi Cube Level p1", 2, 2, true);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Blue Taxi Cube Level p2", 2,2, true);
    List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("180", new PathConstraints(1, 1));

    return new SequentialCommandGroup(intake.deployPiston(),new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), intake.deployPiston(), intake.dropAndStop().alongWith(autoBuilder.fullAuto(pathGroup2)), intake.slowIntake(), intake.retractPiston(), new ChargingStationAuto180(drivetrain), autoBuilder.fullAuto(pathGroup3), intake.fastOutake().alongWith(new ChargingStationAuto(drivetrain)));

  }
  public Command CBlueTaxiCubeCubeLevel(){
    List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("Correct Blue Taxi Wall pt2",5.5,3);
    List<PathPlannerTrajectory> pathGroup4 = PathPlanner.loadPathGroup("Correct Blue Taxi Wall pt3",6.5,5);
    return new SequentialCommandGroup(CBlueTaxiWallCube(),autoBuilder.fullAuto(pathGroup3).alongWith(new WaitCommand(1).andThen(intake.deployPiston().andThen(intake.intake()))),intake.retractPiston(),intake.stopSpin(),autoBuilder.fullAuto(pathGroup4), intake.outtake(), new WaitCommand(.2), intake.stopSpin());
  }

  public Command CBlueTaxiWall(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Correct Blue Taxi Wall", 2, 2);

    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1));
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
    autoBuilder2 = new SwerveAutoBuilder(
      () -> drivetrain.getOdometryPose2dAprilTags(),
      (pose) -> drivetrain.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
      Constants.m_kinematics, // SwerveDriveKinematics
      new PIDConstants(1, 0.01, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(4, 0.1, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      (state) -> drivetrain.autoSetChassisState(state), // Module states consumer used to output to the drive
                                                        // subsystem
      eventMap,
      true,
      drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
  );
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Correct Red Taxi HP", 5, 3, false);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Correct Blue Taxi HP Back", 5, 3,true);
    List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("Correct Blue HP 3 piece pt1",4,2,false);
    List<PathPlannerTrajectory> pathGroup4 = PathPlanner.loadPathGroup("Correct Blue HP 3 piece pt2",4,2,false);
    return new SequentialCommandGroup(intake.deployPiston(),new WaitCommand(.2), autoBuilder2.fullAuto(pathGroup1).alongWith(intake.intake()),intake.retractPiston(), intake.stopSpin(),autoBuilder.fullAuto(pathGroup2), intake.outtake(), new WaitCommand(.2), intake.stopSpin(), autoBuilder2.fullAuto(pathGroup3).alongWith(new WaitCommand(1.5).andThen(intake.deployPiston().andThen(intake.intake()))),intake.retractPiston(),autoBuilder2.fullAuto(pathGroup4), intake.outtake(), new WaitCommand(.3), intake.stopSpin());
  }

  public Command RedHP2Piece(){
    PathPlannerTrajectory pathGroup1 = PathPlanner.loadPath("2 piece one path red hp", 4,3);
    return new SequentialCommandGroup(autoBuilder3.fullAuto(pathGroup1));
    
  }
  public Command RedHP2PieceCharge(){
    PathPlannerTrajectory pathGroup1 = PathPlanner.loadPath("2 piece one path red hp charge", 5,3);
    return new SequentialCommandGroup(autoBuilder3.fullAuto(pathGroup1), new ChargingStationAuto(drivetrain));
    
  }
  public Command RedHP3Piece(){
    PathPlannerTrajectory path1 = PathPlanner.loadPath("3 piece one path red hp", 4,2);
    return new SequentialCommandGroup(autoBuilder3.fullAuto(path1));
  }
  public Command RedHP4Piece(){
    PathPlannerTrajectory path1 = PathPlanner.loadPath("4 piece one path red hp", 5,4.5);
    return new SequentialCommandGroup(autoBuilder3.fullAuto(path1));
  }
  public Command BlueHP2Piece(){
    PathPlannerTrajectory path1 = PathPlanner.loadPath("2 piece one path blue hp", 4,2);
    return new SequentialCommandGroup(autoBuilder3.fullAuto(path1));
  }
  public Command BlueHP3Piece(){
    PathPlannerTrajectory path1 = PathPlanner.loadPath("3 piece one path blue hp", 4,2);
    return new SequentialCommandGroup(autoBuilder3.fullAuto(path1));
    
  }
  

  public Command CRedTaxiWallCube(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Correct Blue Taxi HP", 3, 3, true);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Correct Blue Taxi Wall Back 3 piece", 4, 2.5, true);

    return new SequentialCommandGroup(intake.deployPiston(), autoBuilder.fullAuto(pathGroup1).alongWith(intake.intake()), intake.slowIntake(), intake.retractPiston(), autoBuilder.fullAuto(pathGroup2), intake.outtake(), new WaitCommand(.3), intake.stopSpin());
  }
  public Command test(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("test", 2,1);
    
    return new SequentialCommandGroup(autoBuilder.followPathGroup(pathGroup1));
  }

  public Command CRedTaxiWallCubeCube(){
    // List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Correct Blue Taxi HP", 5, 3, true);
    // List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Correct Blue Taxi HP Back", 3.5,2,true);
    List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("Correct Blue Taxi Wall pt2",3,2,true);
    List<PathPlannerTrajectory> pathGroup4 = PathPlanner.loadPathGroup("Correct Blue Taxi Wall pt3",5.5,5,true);
    return new SequentialCommandGroup(CRedTaxiWallCube(),autoBuilder.fullAuto(pathGroup3).alongWith(new WaitCommand(1).andThen(intake.deployPiston().andThen(intake.intake()))),intake.retractPiston(),intake.stopSpin(),autoBuilder.fullAuto(pathGroup4), intake.outtake(), new WaitCommand(.2), intake.stopSpin());
    // intake.deployPiston(),autoBuilder.fullAuto(pathGroup1).alongWith(intake.intake()),intake.retractPiston(),intake.stopSpin(),autoBuilder.fullAuto(pathGroup2), intake.fastOutake(), new WaitCommand(0.2), intake.stopSpin(), 
  }

  public Command CBlueTaxiWallCube(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Correct Blue Taxi Wall", 4, 3);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Correct Blue Taxi Back Wall", 3,2);

    return new SequentialCommandGroup(intake.deployPiston(), intake.intake().alongWith(autoBuilder.fullAuto(pathGroup1)), intake.slowIntake(), intake.retractPiston(), autoBuilder.fullAuto(pathGroup2), intake.outtake(), new WaitCommand(.3), intake.stopSpin());

  }

  public Command Level(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("SOMETHING", 2,2);

    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), new ChargingStationAuto(drivetrain));
  } 

  public Command TaxiandLevel(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Taxi and Level p1", 1.4,2);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Taxi and Level p2", 1.4,2);


    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), new WaitCommand(1.5), autoBuilder.fullAuto(pathGroup2), new ChargingStationAuto(drivetrain));
  }



  //Wrong Autons
  
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