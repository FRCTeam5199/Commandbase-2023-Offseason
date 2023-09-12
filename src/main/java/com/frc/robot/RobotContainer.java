// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frc.robot;

import com.frc.robot.commands.Auton;
import com.frc.robot.commands.CompressorCommand;
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
import com.frc.robot.utility.TagManager;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

<<<<<<< Updated upstream
=======
  public final TagManager tagManager = new TagManager();

  private final CompositeCommand compositeCommand;

>>>>>>> Stashed changes
  public final Auton auton;

  // final AprilTagManager tagManager = new AprilTagManager();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain = new Drivetrain();
    driveCommand = new DriveCommand(drivetrain, manualControls);
    drivetrain.setDefaultCommand(driveCommand);
    compressor.init();

    claw.init();

    elevator.init();

    arm.init();

    wrist.init();

    intake.init();

    tagManager.print();

    auton = new Auton(drivetrain, arm, intake, elevator, claw, wrist);

    // tagManager.init();

    createControllers();

    configureBindings();

    CompressorCommand compressorRun = new CompressorCommand(compressor);

    compressor.setDefaultCommand(compressorRun);
  }

  private void createControllers() {
    commandXboxController = new CommandXboxController(1);

    buttonPanel = new CommandButtonPanel();
  }

  public void configureBindings() {
    if (Constants.ENABLE_CLAW) {
      // commandXboxController.a().onTrue(claw.openPiston());
      // commandXboxController.y().onTrue(claw.closePiston());
      manualControls.a().onTrue(claw.openPiston());
      manualControls.y().onTrue(claw.closePiston());
    }

    // if (Constants.ENABLE_ELEVATOR && Constants.ARM_ELEVATOR_MANUAL) {
    // commandXboxController.x().whileTrue(elevator.move(1));
    // commandXboxController.b().whileTrue(elevator.move(-1));
    // }

    // Wrist 
    SequentialCommandGroup wristCommandGroup = new SequentialCommandGroup(
        new InstantCommand(() -> wrist.palmUp()),
        new WaitUntilCommand(wrist::isPalmUp),
        new InstantCommand(() -> wrist.stopRotation()),
        new WaitCommand(0.5),
        new InstantCommand(() -> wrist.palmDown()),
        new WaitUntilCommand(wrist::isPalmDown),
        new InstantCommand(() -> wrist.stopRotation())
    );

    // Stable
    ConditionalCommand stableCommandGroup = new ConditionalCommand(
        new SequentialCommandGroup(
          new InstantCommand(() -> elevator.low()),
          new InstantCommand(() -> arm.retract()),
          new InstantCommand(() -> arm.rotateStable())),
        new SequentialCommandGroup(
          new InstantCommand(() -> arm.retract()),
          new InstantCommand(() -> elevator.low()),
          new InstantCommand(() -> arm.rotateStable()),
          new InstantCommand(() -> wrist.palmDown()),
          new WaitUntilCommand(wrist::isPalmDown),
          new InstantCommand(() -> wrist.stopRotation())),
        arm::isFront);


    // Human player
    ConditionalCommand humanPlayerCommandGroup = new ConditionalCommand(
      new SequentialCommandGroup(
        new InstantCommand(() -> elevator.humanPlayer()),
        new InstantCommand(() -> arm.rotateHumanPlayer()),
        new InstantCommand(() -> arm.extendHumanPlayer())),
      new SequentialCommandGroup(
        new InstantCommand(() -> arm.retract()),
        new InstantCommand(() -> elevator.low()),
        new InstantCommand(() -> arm.rotateHumanPlayer()),
        new WaitCommand(1.5),
        new InstantCommand(() -> elevator.humanPlayer()),
        new InstantCommand(() -> arm.extendHumanPlayer()),
        new InstantCommand(() -> wrist.palmDown()),
        new WaitUntilCommand(wrist::isPalmDown),
        new InstantCommand(() -> wrist.stopRotation())),
      arm::isFront);


    // High goal
    ConditionalCommand highGoalCommandGroup = new ConditionalCommand(
      new SequentialCommandGroup(
        new InstantCommand(() -> elevator.low()),
        new InstantCommand(() -> arm.retract()),
        new InstantCommand(() -> arm.rotateHigh()),
        new WaitCommand(1.5),
        new InstantCommand(() -> arm.extend()),
        new InstantCommand(() -> elevator.high()),
        new InstantCommand(() -> wrist.palmUp()),
        new WaitUntilCommand(wrist::isPalmUp),
        new InstantCommand(() -> wrist.stopRotation())
      ),
      new SequentialCommandGroup(
        new InstantCommand(() -> elevator.high()),
        new InstantCommand(() -> arm.rotateHigh()),
        new InstantCommand(() -> arm.extend())
      ),
      arm::isFront);

    // Medium goal
    ConditionalCommand midGoalCommandGroup = new ConditionalCommand(
      new SequentialCommandGroup(
        new InstantCommand(() -> arm.retract()),
        new InstantCommand(() -> elevator.low()),
        new InstantCommand(() -> arm.rotateMedium()),
        new WaitCommand(1.5),
        new InstantCommand(() -> arm.extendMedium()),
        new InstantCommand(() -> elevator.medium()),
        new InstantCommand(() -> wrist.palmUp()),
        new WaitUntilCommand(wrist::isPalmUp),
        new InstantCommand(() -> wrist.stopRotation())
      ),
      new SequentialCommandGroup(
        new InstantCommand(() -> elevator.medium()),
        new InstantCommand(() -> arm.rotateMedium()),
        new InstantCommand(() -> arm.extendMedium())
      ),
      arm::isFront);

    // Low goal
    ConditionalCommand lowGoalCommandGroup = new ConditionalCommand(
      new SequentialCommandGroup(
        new InstantCommand(() -> arm.retract()),
        new InstantCommand(() -> elevator.low()),
        new InstantCommand(() -> arm.rotateLow()),
        new WaitCommand(1.5),
        new InstantCommand(() -> arm.retract()),
        new InstantCommand(() -> wrist.palmUp()),
        new WaitUntilCommand(wrist::isPalmUp),
        new InstantCommand(() -> wrist.stopRotation())
      ),
      new SequentialCommandGroup(
        new InstantCommand(() -> elevator.low()),
        new InstantCommand(() -> arm.rotateLow()),
        new InstantCommand(() -> arm.retract())
      ),
      arm::isFront);


    // Map button triggers
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_2, 12).onTrue(humanPlayerCommandGroup);
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 7).onTrue(stableCommandGroup);
    // buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 7).onTrue(wristCommandGroup);
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 9).onTrue(highGoalCommandGroup);
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 10).onTrue(midGoalCommandGroup);
    buttonPanel.button(Constants.ControllerIds.BUTTON_PANEL_1, 11).onTrue(lowGoalCommandGroup);

    // commandXboxController.povUp().onTrue(humanPlayerCommandGroup);
    // commandXboxController.leftBumper().onTrue(stableCommandGroup);
    // commandXboxController.povLeft().onTrue(highGoalCommandGroup);
    // commandXboxController.povRight().onTrue(mediumGoalCommandGroup);
    // commandXboxController.povDown().onTrue(lowGoalCommandGroup);

    // if (Constants.ENABLE_ARM && Constants.ARM_ELEVATOR_MANUAL) {
    // commandXboxController.povUp().whileTrue(arm.moveRotate(-1));
    // commandXboxController.povDown().whileTrue(arm.moveRotate(1));

    // commandXboxController.povLeft().whileTrue(arm.moveExtend(-20));
    // commandXboxController.povRight().whileTrue(arm.moveExtend(20));
    // }

    if (Constants.WRIST_MANUAL && Constants.ENABLE_WRIST) {
      // commandXboxController.leftBumper().whileTrue(wrist.move(5));
      // commandXboxController.rightBumper().whileTrue(wrist.move(-5));
    }
    // TEMPORARY ELSE STATEMENT REMOVE LATER
    // else {
    // commandXboxController.leftBumper().whileTrue(wrist.setSetpoint(0));
    // commandXboxController.rightBumper().whileTrue(wrist.setSetpoint(5));
    // }

    if (Constants.ENABLE_INTAKE) {
      if (Constants.INTAKE_MANUAL) {
        manualControls.b().onTrue(intake.spinOutakeOnBottom(false)).onFalse(intake.spinOutakeOnBottom(true));

        manualControls.x().toggleOnTrue(intake.spinBottomWithLimit());

        // commandXboxController.x().toggleOnTrue(intake.spinBottomWithLimit());
        // // commandXboxController.x().onTrue(intake.deployPiston());
        // // commandXboxController.b().onTrue(intake.retractPiston());

        // commandXboxController.b().onTrue(intake.spinOutakeOnBottom(false));
        // commandXboxController.b().onFalse(intake.spinOutakeOnBottom(true));
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
    return auton.TaxiCubeLevel180();
  }

}
