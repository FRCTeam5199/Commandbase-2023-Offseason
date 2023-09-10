// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frc.robot;

import com.frc.robot.commands.Auton;
import com.frc.robot.commands.CompositeCommand;
import com.frc.robot.commands.CompressorCommand;

// import java.io.File;

import com.frc.robot.commands.DriveCommand;
import com.frc.robot.controls.ManualControls;
import com.frc.robot.controls.customcontrollers.CommandButtonPanel;
import com.frc.robot.subsystems.CompressorSubsystem;
import com.frc.robot.subsystems.Drivetrain;
import com.frc.robot.subsystems.piecemanipulation.ArmSubsystem;
import com.frc.robot.subsystems.piecemanipulation.ClawSubsystem;
import com.frc.robot.subsystems.piecemanipulation.ElevatorSubsystem;
import com.frc.robot.subsystems.piecemanipulation.IntakeSubsystem;
import com.frc.robot.subsystems.piecemanipulation.WristSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  final Drivetrain drivetrain;
  private final DriveCommand driveCommand;
  private final XboxController driveXboxController = new XboxController(0);
  private final ManualControls manualControls = new ManualControls(driveXboxController);

  public CommandXboxController commandXboxController;
  public CommandButtonPanel buttonPanel;
  
  // not public or private so Robot.java has access to it.
  public final static ArmSubsystem arm = new ArmSubsystem();

  public static ElevatorSubsystem elevator = new ElevatorSubsystem();

  public static WristSubsystem wrist = new WristSubsystem();

  public static final ClawSubsystem claw = new ClawSubsystem();

  public static final IntakeSubsystem intake = new IntakeSubsystem();

  public final CompressorSubsystem compressor = new CompressorSubsystem();

  private final CompositeCommand compositeCommand;

  public final Auton auton;

  private enum ArmPositionStateSelector {
    ARMFRONT,
    ARMBACK
  }

  // final AprilTagManager tagManager = new AprilTagManager();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain = new Drivetrain();
    driveCommand = new DriveCommand(drivetrain, manualControls);
    drivetrain.setDefaultCommand(driveCommand);

    compositeCommand = new CompositeCommand(elevator, arm);

    compressor.init();

    claw.init();

    elevator.init();

    arm.init();

    wrist.init();

    intake.init();

    auton = new Auton(drivetrain, arm, intake, elevator, claw, wrist);

    // tagManager.init();

    createControllers();

    configureBindings();

    CompressorCommand compressorRun = new CompressorCommand(compressor);

    compressor.setDefaultCommand(compressorRun);
  }

// Determine which command to run based on robot state
  private ArmPositionStateSelector select() {
    if (arm.isFront()) {
      System.out.println("Arm Is Front");
      return ArmPositionStateSelector.ARMFRONT;
    } else {
      System.out.println("Arm Is Back");
      return ArmPositionStateSelector.ARMBACK;
    }
  }

  private void createControllers() {
    commandXboxController = new CommandXboxController(1);

    buttonPanel = new CommandButtonPanel();
  }
      

  public void configureBindings() {
    if (Constants.ENABLE_CLAW) {
      commandXboxController.a().onTrue(claw.openPiston());
      commandXboxController.y().onTrue(claw.closePiston());
    }

    // if (Constants.ENABLE_ELEVATOR && Constants.ARM_ELEVATOR_MANUAL) {
    // commandXboxController.x().whileTrue(elevator.move(1));
    // commandXboxController.b().whileTrue(elevator.move(-1));
    // }

    // Stable
    ParallelCommandGroup stableCommandGroup = new ParallelCommandGroup();
    stableCommandGroup.addCommands(
        // (new SelectCommand(
        //   // Maps selector values to commands
        //   Map.ofEntries(
        //       Map.entry(ArmPositionStateSelector.ARMFRONT, new WaitCommand(0)),
        //       Map.entry(ArmPositionStateSelector.ARMBACK, new SequentialCommandGroup(
        //           new InstantCommand(() -> arm.retract()),
        //           new InstantCommand(() -> elevator.low()),
        //           new WaitUntilCommand(() -> arm.isRetracted()),
        //           new InstantCommand(() -> arm.rotateBack())))),
        //   this::select)),
        new InstantCommand(() -> arm.rotateFront()),
        new InstantCommand(() -> elevator.low()),
        new InstantCommand(() -> arm.retract()));

    // Human player
    ParallelCommandGroup humanPlayerCommandGroup = new ParallelCommandGroup();
    humanPlayerCommandGroup.addCommands(
        // (new SelectCommand(
        //   // Maps selector values to commands
        //   Map.ofEntries(
        //       Map.entry(ArmPositionStateSelector.ARMFRONT, new WaitCommand(0)),
        //       Map.entry(ArmPositionStateSelector.ARMBACK, new SequentialCommandGroup(
        //           new InstantCommand(() -> arm.retract()),
        //           new InstantCommand(() -> elevator.low()),
        //           new WaitUntilCommand(() -> arm.isRetracted()),
        //           new InstantCommand(() -> arm.rotateBack())))),
        //   this::select)),
        new InstantCommand(() -> elevator.humanplayer()),
        new InstantCommand(() -> arm.rotateHumanplayer()),
        new InstantCommand(() -> arm.extendHumanplayer()));
        // new InstantCommand(() -> wrist.move(1)),
        // new WaitUntilCommand(() -> wrist.isFlipped()),
        // new InstantCommand(() -> wrist.move(0)),
        // new PrintCommand("Moved"));

    // High goal
    ParallelCommandGroup highGoalCommandGroup = new ParallelCommandGroup();
    highGoalCommandGroup.addCommands(
        // (new SelectCommand(
        //   // Maps selector values to commands
        //   Map.ofEntries(
        //       Map.entry(ArmPositionStateSelector.ARMFRONT, new SequentialCommandGroup(
        //           new InstantCommand(() -> arm.retract()),
        //           new InstantCommand(() -> elevator.low()),
        //           new WaitUntilCommand(() -> arm.isRetracted()),
        //           new InstantCommand(() -> arm.rotateFront()))),
        //       Map.entry(ArmPositionStateSelector.ARMBACK, new WaitCommand(0))),
        //   this::select)),
        new InstantCommand(() -> elevator.high()),
        new InstantCommand(() -> arm.rotateHigh()),
        new InstantCommand(() -> arm.extend()));

    // Medium goal
    ParallelCommandGroup mediumGoalCommandGroup = new ParallelCommandGroup();
    mediumGoalCommandGroup.addCommands(
        // (new SelectCommand(
        //   // Maps selector values to commands
        //   Map.ofEntries(
        //       Map.entry(ArmPositionStateSelector.ARMFRONT, new SequentialCommandGroup(
        //           new InstantCommand(() -> arm.retract()),
        //           new InstantCommand(() -> elevator.low()),
        //           new WaitUntilCommand(() -> arm.isRetracted()),
        //           new InstantCommand(() -> arm.rotateFront()))),
        //       Map.entry(ArmPositionStateSelector.ARMBACK, new WaitCommand(0))),
        //   this::select)),
        new InstantCommand(() -> elevator.medium()),
        new InstantCommand(() -> arm.rotateMedium()),
        new InstantCommand(() -> arm.extendMedium()));

    // Low goal
    ParallelCommandGroup lowGoalCommandGroup = new ParallelCommandGroup();
    lowGoalCommandGroup.addCommands(
        // (new SelectCommand(
        //   // Maps selector values to commands
        //   Map.ofEntries( w
        //       Map.entry(ArmPositionStateSelector.ARMFRONT, new SequentialCommandGroup(
        //           new InstantCommand(() -> arm.retract()),
        //           new InstantCommand(() -> elevator.low()),
        //           new WaitUntilCommand(() -> arm.isRetracted()),
        //           new InstantCommand(() -> arm.rotateFront()))),
        //       Map.entry(ArmPositionStateSelector.ARMBACK, new WaitCommand(0))),
        //   this::select)),
        new InstantCommand(() -> elevator.low()),
        new InstantCommand(() -> arm.rotateLow()),
        new InstantCommand(() -> arm.retract()));

    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_2, 12).onTrue(humanPlayerCommandGroup);
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 7).onTrue(stableCommandGroup);
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 9).onTrue(highGoalCommandGroup);
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 10).onTrue(mediumGoalCommandGroup);
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 11).onTrue(lowGoalCommandGroup);

    // driveXboxController.a().onTrue(() -> System.out.println());
    // commandXboxController.povUp().onTrue(humanPlayerCommandGroup);
    // commandXboxController.leftBumper().onTrue(stableCommandGroup);
    // commandXboxController.povLeft().onTrue(highGoalCommandGroup);
    // commandXboxController.povRight().onTrue(mediumGoalCommandGroup);
    // commandXboxController.povDown().onTrue(lowGoalCommandGroup);

    // commandXboxController.b().onTrue(humanPlayerCommandGroup);

    // new InstantCommand(()->arm.rotateForward()),
    // new InstantCommand(()->arm.extend())
    // if (Constants.ENABLE_ARM && Constants.ARM_ELEVATOR_MANUAL) {
    //   commandXboxController.povUp().whileTrue(arm.moveRotate(-1));
    //   commandXboxController.povDown().whileTrue(arm.moveRotate(1));

    //   commandXboxController.povLeft().whileTrue(arm.moveExtend(-20));
    //   commandXboxController.povRight().whileTrue(arm.moveExtend(20));
    // }

    if (Constants.WRIST_MANUAL && Constants.ENABLE_WRIST) {
      // commandXboxController.leftBumper().whileTrue(wrist.move(5));
      // commandXboxController.rightBumper().whileTrue(wrist.move(-5));
    }
    // TEMPORARY ELSE STATEMENT REMOVE LATER
    // else {
    //   commandXboxController.leftBumper().whileTrue(wrist.setSetpoint(0));
    //   commandXboxController.rightBumper().whileTrue(wrist.setSetpoint(5));
    // }

    if (Constants.ENABLE_INTAKE) {
      if (Constants.INTAKE_MANUAL) {
        commandXboxController.x().toggleOnTrue(intake.spinBottomWithLimit());
        // commandXboxController.x().onTrue(intake.deployPiston());
        // commandXboxController.b().onTrue(intake.retractPiston());

        commandXboxController.b().onTrue(intake.spinOutakeOnBottom(false));
        commandXboxController.b().onFalse(intake.spinOutakeOnBottom(true));
      } else {
        buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 6).toggleOnTrue(intake.spinBottomWithLimit());

        buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 4).onTrue(intake.deployPiston());

        buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 3).onTrue(intake.retractPiston());
      }
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auton.TaxiWallCubeShoot();
  }

}
