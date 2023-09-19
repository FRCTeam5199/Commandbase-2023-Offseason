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
import com.pathplanner.lib.commands.FollowPathWithEvents;
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
        () -> drivetrain.getOdometryPose2dNoApriltags(), // Pose2d supplier TODO: possibly revert back to no apriltags
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
        () -> drivetrain.getPose(), // Pose2d supplier
        (pose) -> drivetrain.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.m_kinematics, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(1.0, 0.1, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        (state) -> drivetrain.autoSetChassisState(state), // Module states consumer used to output to the drive
                                                          // subsystem
        eventMap,
        false,
        drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
    );
    //Autons must be manually entered into the auton chooser
    autonChooser.setDefaultOption("Red Taxi Cube Level", RedTaxiCubeLevel());
    autonChooser.addOption("Blue Taxi Cube Level", BlueTaxiCubeLevel());
    autonChooser.addOption("Red Taxi Wall", RedTaxiWall());
    autonChooser.addOption("Blue Taxi Wall", BlueTaxiWall());
    autonChooser.addOption("Red Taxi Wall Cube", RedTaxiWallCube());
    autonChooser.addOption("Red Taxi Wall Cube Shoot", RedTaxiWallCubeShoot());
    autonChooser.addOption("Blue Taxi Wall Cube Shoot", BlueTaxiWallCubeShoot());
    autonChooser.addOption("Red Taxi Cube Level 180", RedTaxiCubeLevel180());
    autonChooser.addOption("Blue Taxi Cube Level 180", BlueTaxiCubeLevel180());
    autonChooser.addOption("Red Taxi Over Charge", RedTaxiOverCharge());
    autonChooser.addOption("Blue Taxi Over Charge", BlueTaxiOverCharge());
    autonChooser.addOption("Red Taxi Over Charge Level", RedTaxiOverChargeLevel());
    autonChooser.addOption("Blue Taxi Over Charge Level", BlueTaxiOverChargeLevel());
    autonChooser.addOption("Blue Taxi HP Cube Shoot", BlueTaxiHPCubeShoot());
    autonChooser.addOption("Red Taxi Wall Cube Shoot Cube", RedTaxiWallCubeShootCube());
    autonChooser.addOption("Red Taxi Wall Cube Shoot Cube Shoot", RedTaxiWallCubeShootCubeShoot());
    autonChooser.addOption("Red Taxi HP", RedTaxiHP());
    autonChooser.addOption("Red Taxi HP Cube", RedTaxiHPCube());
  

    Shuffleboard.getTab("Auton").add("Auton Style",autonChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1);;

  }

  public Command getAuton(){
    return autonChooser.getSelected();

  }
  
  public Command getAutoCommand(String pathName) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(1.5, 0.75));

    return autoBuilder.fullAuto(pathGroup);
  }

  public Command TaxiandLevel(){
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Taxi and Level Red", new PathConstraints(1.5, 2));

    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2 ), intake.retractPiston(), autoBuilder.fullAuto(pathGroup.get(0)), new ChargingStationAuto(drivetrain));
  }

  
  public Command RedTaxiCubeLevel(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("TaxiOverCharge Red", new PathConstraints(2, 2));
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("ChargeFromPiece Red", new PathConstraints(1.5, 2));
    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), intake.deployPiston(), intake.dropAndStop().alongWith(autoBuilder.fullAuto(pathGroup2)), new ChargingStationAuto(drivetrain));
  }

  public Command BlueTaxiCubeLevel(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("TaxiOverCharge Red", 2, 2, true);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("ChargeFromPiece Red", 2,2,true);

    return new SequentialCommandGroup(intake.deployPiston(),new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), intake.deployPiston(), intake.dropAndStop().alongWith(autoBuilder.fullAuto(pathGroup2)), new ChargingStationAuto(drivetrain));

  }

  public Command RedTaxiCubeLevel180(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("TaxiOverCharge Red", new PathConstraints(2, 2));
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("ChargeFromPiece Red", new PathConstraints(1.7, 2));
    List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("180", new PathConstraints(1, 1));


    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), intake.deployPiston(), intake.dropAndStop().alongWith(autoBuilder.fullAuto(pathGroup2)), intake.slowIntake(), intake.retractPiston(), new ChargingStationAuto180(drivetrain), autoBuilder.fullAuto(pathGroup3),intake.fastOutake().alongWith(new ChargingStationAuto(drivetrain)));
  }

  public Command BlueTaxiCubeLevel180(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("TaxiOverCharge Red", 2, 2, true);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("ChargeFromPiece Red", 1.7, 2, true);
    List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("180", new PathConstraints(1, 1));


    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), intake.deployPiston(), intake.dropAndStop().alongWith(autoBuilder.fullAuto(pathGroup2)), intake.slowIntake(), intake.retractPiston(), new ChargingStationAuto180(drivetrain), autoBuilder.fullAuto(pathGroup3),intake.fastOutake().alongWith(new ChargingStationAuto(drivetrain)));
  }
  

  public Command RedTaxiOverCharge(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("TaxiOverCharge Red", new PathConstraints(1, 1));
    return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1));
  }

  public Command BlueTaxiOverCharge(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("TaxiOverCharge Blue", new PathConstraints(1, 1));
    return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1));
  }

  public Command RedTaxiOverChargeLevel(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Taxi and Level Red", new PathConstraints(1, 1));
    return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1), new ChargingStationAuto(drivetrain));
  }
  public Command BlueTaxiOverChargeLevel(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Taxi and Level Blue", new PathConstraints(1, 1));
    return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1), new ChargingStationAuto(drivetrain));
  }
  
  /**
   * only goes foward and cube shooter
   * @return
   */
  public Command RedTaxiWall() {
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall", new PathConstraints(1.25, 0.9));
    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(1),intake.retractPiston(), autoBuilder.fullAuto(pathGroup1.get(0)));
  }
  
  /**
   * 
   * @return only moves foward and backward on side of wall cube shooter
   */
  public Command RedTaxiWallCube(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall", new PathConstraints(1.25, 0.9));
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Red Taxi Wall Back", new PathConstraints(1.25, 1));
    
    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(1), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1.get(0)), autoBuilder.fullAuto(pathGroup2.get(0)));
  }
  /**
   * 
   * @return moves foward and backwards cube shooter and picks up cube and scores
   */

  public Command RedTaxiWallCubeShoot(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall", new PathConstraints(4,4));
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Red Taxi Wall Back Turn End dif", new PathConstraints(3, 1.5));
    
    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.25), autoBuilder.fullAuto(pathGroup1.get(0)).alongWith(intake.intake()), intake.retractPiston(), autoBuilder.fullAuto(pathGroup2.get(0)), intake.outtake(), new WaitCommand(.50), intake.stopSpin());
  }

  public Command BlueTaxiWallCubeShoot() {
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Blue Taxi Wall", 2, 1, true);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Red Taxi Wall Back Turn End", 2, 1.5, true);

    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.25), autoBuilder.fullAuto(pathGroup1.get(0)).alongWith(intake.intake()), intake.retractPiston(), autoBuilder.fullAuto(pathGroup2.get(0)), intake.outtake(), new WaitCommand(.50), intake.stopSpin());
  }

  public Command RedTaxiWallCubeShootCube(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall", new PathConstraints(5,5));
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Red Taxi Wall Back Turn End dif", new PathConstraints(2, 1.5));
    List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("Red Taxi Wall Back Turn cube", new PathConstraints(3, 1.5));
    
    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.25), autoBuilder.fullAuto(pathGroup1.get(0)).alongWith(intake.intake()), 
    intake.retractPiston(), autoBuilder.fullAuto(pathGroup2.get(0)), intake.outtake(), new WaitCommand(.50), intake.stopSpin(), autoBuilder.fullAuto(pathGroup3.get(0)).alongWith(new WaitCommand(3).andThen(intake.deployPiston()).andThen(intake.intake())), intake.retractPiston());
  }
  public Command RedTaxiWallCubeShootCubeShoot(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall", 7,4, false);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Red Taxi Wall Back Turn End dif", 3, 1.5, false);
    List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("Red Taxi Wall Back Turn cube", 4, 2, false);
    List<PathPlannerTrajectory> pathGroup4 = PathPlanner.loadPathGroup("Red Taxi Wall Back Turn cube end auto", 7, 6, false);
    List<PathPlannerTrajectory> pathGroup5 = PathPlanner.loadPathGroup("Red Taxi Wall Back Turn cube end auto spin", 7, 7, false);
    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.25), autoBuilder.fullAuto(pathGroup1.get(0)).alongWith(intake.intake()), 
    intake.retractPiston(), autoBuilder.fullAuto(pathGroup2.get(0)), intake.outtake(), new WaitCommand(.50), intake.stopSpin(), autoBuilder.fullAuto(pathGroup3.get(0)).alongWith(new WaitCommand(3).andThen(intake.deployPiston()).andThen(intake.intake())), intake.retractPiston(),
    autoBuilder.fullAuto(pathGroup4.get(0)), intake.fastOutake(), autoBuilder.fullAuto(pathGroup5.get(0)), intake.stopSpin());
    
  }
  public Command RedTaxiHP(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi HP",3, 2, false);
    return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1.get(0)));
  }
  public Command RedTaxiHPCube(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi HP",3, 2, false);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Red Taxi HP Back",3, 2, false);
    
    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(0.25), autoBuilder.fullAuto(pathGroup1.get(0)).alongWith(intake.intake()),intake.retractPiston(), autoBuilder.fullAuto(pathGroup2.get(0)), intake.fastOutake(), new WaitCommand(0.25), intake.stopSpin());
  }
  public Command BlueTaxiWall(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Blue Taxi Wall",3, 1.5, true);
    return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1.get(0)));
  }
  public Command BlueTaxiHP(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Blue Taxi HP",new PathConstraints(1.25, 0.9));
    return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1.get(0)));
  }  

  public Command BlueTaxiHPCubeShootFake(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall",3, 3, true);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Blue Taxi Wall Back Turn End dif",2, 1.5, true);
    return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1.get(0)), autoBuilder.fullAuto(pathGroup2.get(0)));
  }
  public Command BlueTaxiHPCubeShoot(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall",3, 3, true);
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Blue Taxi Wall Back Turn End dif",2, 1.5, true);
    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.25), autoBuilder.fullAuto(pathGroup1.get(0)).alongWith(intake.intake()), intake.retractPiston(), autoBuilder.fullAuto(pathGroup2.get(0)), intake.fastOutake(), new WaitCommand(.50), intake.stopSpin());
  }


  public Command getTaxiCharge(){
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("TaxiCharge", new PathConstraints(2, 1));

    return new SequentialCommandGroup(
      autoBuilder.fullAuto(pathGroup.get(0)),
      new ChargingStationAuto(drivetrain)
    );
  }


  public Command getCharge(){
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Charge", new PathConstraints(1, 1));

    return new SequentialCommandGroup(
      intake.deployPiston(), new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup.get(0)),
      new ChargingStationAuto(drivetrain)
    );
  }






  
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