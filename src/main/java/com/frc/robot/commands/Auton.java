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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
        () -> drivetrain.getOdometryPose2dAprilTags(), // Pose2d supplier TODO: possibly revert back to no apriltags
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
    autonChooser.setDefaultOption("Taxi Wall", TaxiWall());
    autonChooser.addOption("Taxi and Level", TaxiandLevel());
    autonChooser.addOption("Taxi Cube Level", TaxiCubeLevel());
    autonChooser.addOption("Taxi Cube Level 180", TaxiCubeLevel180());
    autonChooser.addOption("Taxi Wall Cube", TaxiWallCube());
    autonChooser.addOption("Taxi Wall Cube Shoot", TaxiWallCubeShoot());

    Shuffleboard.getTab("Auton").add("Auton Style", autonChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1);;

  }

  public Command getAuton() {
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

  public Command TaxiCubeLevel(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("TaxiOverCharge Red", new PathConstraints(2, 2));
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("ChargeFromPiece", new PathConstraints(1.5, 2));
    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), intake.deployPiston(), intake.dropAndStop().alongWith(autoBuilder.fullAuto(pathGroup2)), new ChargingStationAuto(drivetrain));
  }

  public Command TaxiCubeLevel180(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("TaxiOverCharge Red", new PathConstraints(2, 2));
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("ChargeFromPiece", new PathConstraints(1.7, 2));
    List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup("180", new PathConstraints(1, 1));

    double initHeading = drivetrain.getHeading();


    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.2), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1), new ZeroHeading(drivetrain), intake.deployPiston(), intake.dropAndStop().alongWith(autoBuilder.fullAuto(pathGroup2)), intake.retractPiston(), new ChargingStationAuto180(drivetrain), autoBuilder.fullAuto(pathGroup3),intake.autonOuttake().alongWith(new ChargingStationAuto(drivetrain)));
  }


  
  // public Command TaxiCubeShootWall(){
  //   List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall", new PathConstraints(1.25, 0.9));
  //   List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Red Taxi Wall Back", new PathConstraints(1.25, 1));
    
  //   return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup1.get(0)), autoBuilder.fullAuto(pathGroup2.get(0)));
  // }
  /**
   * only goes foward and cube shooter
   * @return
   */
  public Command TaxiWall() {
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall", new PathConstraints(1.25, 0.9));
    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(1),intake.retractPiston(), autoBuilder.fullAuto(pathGroup1.get(0)));
  }
  
  /**
   * 
   * @return only moves foward and backward on side of wall cube shooter
   */
  public Command TaxiWallCube(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall", new PathConstraints(1.25, 0.9));
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Red Taxi Wall Back", new PathConstraints(1.25, 1));
    
    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(1), intake.retractPiston(), autoBuilder.fullAuto(pathGroup1.get(0)), autoBuilder.fullAuto(pathGroup2.get(0)));
  }
  /**
   * 
   * @return moves foward and backwards cube shooter and picks up cube and scores
   */
  public Command TaxiWallCubeShoot(){
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Red Taxi Wall", new PathConstraints(1.25, 0.9));
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup("Red Taxi Wall Back Turn End", new PathConstraints(1.25, 1));
    
    return new SequentialCommandGroup(intake.deployPiston(), new WaitCommand(.25), autoBuilder.fullAuto(pathGroup1.get(0)).alongWith(intake.intake()), intake.retractPiston(), autoBuilder.fullAuto(pathGroup2.get(0)), intake.outtake(), new WaitCommand(.50), intake.stopSpin());
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
      autoBuilder.fullAuto(pathGroup.get(0)),
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