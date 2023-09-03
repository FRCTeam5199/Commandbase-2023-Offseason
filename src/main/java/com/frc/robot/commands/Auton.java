package com.frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.List;
import java.util.HashMap;
import java.util.List;

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
import com.frc.robot.Constants;
import com.frc.robot.subsystems.CompressorSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
    eventMap.put("printCommand", new PrintCommand("test Print command"));
    // eventMap.put("PLACE_CONE_HIGH", new RunCommand( () ->
    // arm.setArmState(ArmPose.HIGH_CONE), arm).until( () ->
    // arm.isArmAtSetpoint()));


    autoBuilder = new SwerveAutoBuilder(
        () -> drivetrain.getOdometryPose2dNoApriltags(), // Pose2d supplier TODO: possibly revert back to no apriltags
        (pose) -> drivetrain.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.m_kinematics, // SwerveDriveKinematics
        new PIDConstants(3.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
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
        Constants.m_kinematics, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.6, 0.1, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        (state) -> drivetrain.autoSetChassisState(state), // Module states consumer used to output to the drive
                                                          // subsystem
        eventMap,
        false,
        drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
    );

  }
  
  public Command getAutoCommand(String pathName) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(1.5, 0.75));

    return autoBuilder.fullAuto(pathGroup);
  }

  public Command TaxiandLevel(){
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Taxi and Level", new PathConstraints(2, 1));

    return new SequentialCommandGroup(intake.deployPiston(), intake.retractPiston(), autoBuilder.fullAuto(pathGroup.get(0)), new ChargingStationAuto(drivetrain));
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