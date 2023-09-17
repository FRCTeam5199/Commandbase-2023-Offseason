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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  public final TagManager tagManager = new TagManager();

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
    //commandXboxController = new CommandXboxController(1);

    buttonPanel = new CommandButtonPanel();
  }

  public void configureBindings() {
    // Stable command composition
    ConditionalCommand stableCommandGroup = 
    new ConditionalCommand(
        new ParallelCommandGroup(
          new InstantCommand(() -> elevator.low()),
          new InstantCommand(() -> arm.extendStable()),
          new InstantCommand(() -> arm.rotateStable())
        ),
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new InstantCommand(() -> arm.extendStable()),
            new InstantCommand(() -> elevator.low()),
            new InstantCommand(() -> arm.rotateStable())
          ),
          new SequentialCommandGroup(
            new InstantCommand(() -> wrist.moveLeft()),
            new WaitCommand(0.5),
            new InstantCommand(() -> wrist.stopRotation())
          )
        ),
      arm::isFront
    );

    // Human player command composition
    ConditionalCommand humanPlayerCommandGroup = 
    new ConditionalCommand(
      new ParallelCommandGroup(
        new InstantCommand(() -> elevator.humanPlayer()),
        new InstantCommand(() -> arm.rotateHumanPlayer()),
        new InstantCommand(() -> arm.extendHumanPlayer())
      ),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(() -> arm.extendStable()),
          new InstantCommand(() -> elevator.low()),
          new InstantCommand(() -> arm.rotateHumanPlayer()),
          new WaitCommand(0.8),
          new InstantCommand(() -> elevator.humanPlayer()),
          new InstantCommand(() -> arm.extendHumanPlayer())
        ),
        new SequentialCommandGroup(
            new InstantCommand(() -> wrist.moveLeft()),
            new WaitCommand(0.5),
            new InstantCommand(() -> wrist.stopRotation())
        )
      ),
      arm::isFront
    );

    // High goal command composition
    ConditionalCommand highGoalCommandGroup = 
      new ConditionalCommand(
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new InstantCommand(() -> elevator.low()),
            new InstantCommand(() -> arm.extendStable()),
            new InstantCommand(() -> arm.rotateHigh()),
            new WaitCommand(/*0.8*/0.1),
            new InstantCommand(() -> arm.extendHigh()),
            new InstantCommand(() -> elevator.high())
          ),
          new SequentialCommandGroup(
              new InstantCommand(() -> wrist.moveRight()),
              new WaitCommand(0.5),
              new InstantCommand(() -> wrist.stopRotation())
          )
        ),
        new ParallelCommandGroup(
          new InstantCommand(() -> elevator.high()),
          new InstantCommand(() -> arm.rotateHigh()),
          new InstantCommand(() -> arm.extendHigh())
        ),
      arm::isFront);

    // Medium goal command composition
    ConditionalCommand midGoalCommandGroup = 
      new ConditionalCommand(
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new InstantCommand(() -> arm.extendStable()),
            new InstantCommand(() -> elevator.low()),
            new InstantCommand(() -> arm.rotateMid()),
            new WaitCommand(0.8),
            new InstantCommand(() -> arm.extendMid()),
            new InstantCommand(() -> elevator.mid())
          ),
          new SequentialCommandGroup(
              new InstantCommand(() -> wrist.moveRight()),
              new WaitCommand(0.5),
              new InstantCommand(() -> wrist.stopRotation())
          )
        ),
        new ParallelCommandGroup(
          new InstantCommand(() -> elevator.mid()),
          new InstantCommand(() -> arm.rotateMid()),
          new InstantCommand(() -> arm.extendMid())
        ),
      arm::isFront);

    // Low goal command composition
    ConditionalCommand lowGoalCommandGroup = 
      new ConditionalCommand(
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new InstantCommand(() -> arm.extendStable()),
            new InstantCommand(() -> elevator.low()),
            new InstantCommand(() -> arm.rotateLow()),
            new WaitCommand(0.8),
            new InstantCommand(() -> arm.extendStable())
          ),
          new SequentialCommandGroup(
              new InstantCommand(() -> wrist.moveRight()),
              new WaitCommand(0.5),
              new InstantCommand(() -> wrist.stopRotation())
          )
        ),
        new ParallelCommandGroup(
          new InstantCommand(() -> elevator.low()),
          new InstantCommand(() -> arm.rotateLow()),
          new InstantCommand(() -> arm.extendStable())
        ),
        arm::isFront
      );


    // Map position commands to button panel triggers
    buttonPanel.button(CompConstants.ControllerIds.BUTTON_PANEL_2, 12).onTrue(humanPlayerCommandGroup);
    buttonPanel.button(CompConstants.ControllerIds.BUTTON_PANEL_1, 7).onTrue(stableCommandGroup);
    buttonPanel.button(CompConstants.ControllerIds.BUTTON_PANEL_1, 9).onTrue(highGoalCommandGroup);
    buttonPanel.button(CompConstants.ControllerIds.BUTTON_PANEL_1, 10).onTrue(midGoalCommandGroup);
    buttonPanel.button(CompConstants.ControllerIds.BUTTON_PANEL_1, 11).onTrue(lowGoalCommandGroup);

    buttonPanel.button(CompConstants.ControllerIds.BUTTON_PANEL_1, 4).onTrue(intake.deployPiston());
    buttonPanel.button(CompConstants.ControllerIds.BUTTON_PANEL_1, 3).onTrue(intake.retractPiston());

    // Map claw commands toxbox controler triggers
    if (CompConstants.Piecemanipulation.ENABLE_CLAW) {
      manualControls.a().onTrue(claw.openPiston());
      manualControls.y().onTrue(claw.closePiston());
    }

    // Map claw commands toxbox controler triggers
    if (CompConstants.Piecemanipulation.ENABLE_INTAKE) {
      // manualControls.b().onTrue(intake.spinOutakeOnBottom(false)).onFalse(intake.spinOutakeOnBottom(true));
      // manualControls.x().toggleOnTrue(intake.spinBottomWithLimit());

      // manualCotntrols.b().on
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
