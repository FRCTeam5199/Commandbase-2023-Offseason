// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frc.robot;

import com.frc.robot.commands.Auton;
import com.frc.robot.commands.CompressorCommand;

// import java.io.File;

import com.frc.robot.commands.DriveCommand;
import com.frc.robot.controls.ManualControls;
import com.frc.robot.subsystems.CompressorSubsystem;
import com.frc.robot.subsystems.Drivetrain;
import com.frc.robot.subsystems.piecemanipulation.ArmSubsystem;
import com.frc.robot.subsystems.piecemanipulation.ClawSubsystem;
import com.frc.robot.subsystems.piecemanipulation.ElevatorSubsystem;
import com.frc.robot.subsystems.piecemanipulation.IntakeSubsystem;
import com.frc.robot.subsystems.piecemanipulation.WristSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
        // The robot's subsystems and commands are defined here...

        // not public or private so Robot.java has access to it.
        public final static ArmSubsystem arm = new ArmSubsystem();;
        
        public static ElevatorSubsystem elevator = new ElevatorSubsystem();
        
        public static WristSubsystem wrist = new WristSubsystem();;
        
        public static final ClawSubsystem claw = new ClawSubsystem();
        
        public static final IntakeSubsystem intake = new IntakeSubsystem();
        
        public final CompressorSubsystem compressor = new CompressorSubsystem();
        
        // final AprilTagManager tagManager = new AprilTagManager();

        final Drivetrain drivetrain;
        private final DriveCommand driveCommand;
        private final ManualControls manualControls = new ManualControls(new XboxController(0));
        
        CommandXboxController commandXboxController = new CommandXboxController(1);



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
      
          // tagManager.init();

          configureBindings();
      
          CompressorCommand compressorRun = new CompressorCommand(compressor);

          compressor.setDefaultCommand(compressorRun);

        }

        public void configureBindings() {
          if (Constants.ENABLE_CLAW) {
            commandXboxController.a().onTrue(claw.openPiston());
            commandXboxController.y().onTrue(claw.closePiston());
          }

          if (Constants.ENABLE_ELEVATOR && Constants.ARM_ELEVATOR_MANUAL ) {
            commandXboxController.x().whileTrue(elevator.move(1));
            commandXboxController.b().whileTrue(elevator.move(-1));
          }
          // TEMPORARY ELSE STATEMENT REMOVE LATER
          else {
            commandXboxController.x().whileTrue(elevator.setSetpoint(5));
            commandXboxController.b().whileTrue(elevator.setSetpoint(30));
          }
      
          if (Constants.ENABLE_ARM && Constants.ARM_ELEVATOR_MANUAL) {
            commandXboxController.povUp().whileTrue(arm.moveRotate(-1));
            commandXboxController.povDown().whileTrue(arm.moveRotate(1));
            
            commandXboxController.povLeft().whileTrue(arm.moveExtend(-20));
            commandXboxController.povRight().whileTrue(arm.moveExtend(20));
          }
          // TEMPORARY ELSE STATEMENT REMOVE LATER
          else {
            commandXboxController.povUp().whileTrue(arm.setRotateSetpoint(-20));
            commandXboxController.povDown().whileTrue(arm.setRotateSetpoint(30));
      
            commandXboxController.povLeft().whileTrue(arm.setExtendSetpoint(0));
            commandXboxController.povRight().whileTrue(arm.setExtendSetpoint(21));
          }
      
          if (Constants.WRIST_MANUAL && Constants.ENABLE_WRIST) {
            // commandXboxController.leftBumper().whileTrue(wrist.move(0.5f));
            // commandXboxController.rightBumper().whileTrue(wrist.move(-0.5f));
            commandXboxController.leftBumper().whileTrue(wrist.move(5));
            commandXboxController.rightBumper().whileTrue(wrist.move(-5));
          }
          if (Constants.INTAKE_MANUAL && Constants.ENABLE_INTAKE) {
              commandXboxController.x().onTrue(intake.spinBottomWithLimit());
      
              commandXboxController.b().onTrue(intake.spinOutakeOnBottom(false));
              commandXboxController.b().onFalse(intake.spinOutakeOnBottom(true));
            // }
            // else {
            //   commandXboxController.x().onTrue(intake.victorRunBottomIntake());
            //   commandXboxController.x().onFalse(intake.StopBottomIntake());
            //   commandXboxController.b().onTrue(intake.spinOutakeOnBottom(false));
            //   commandXboxController.b().onFalse(intake.spinOutakeOnBottom(true));
            // }
          }
        }
      

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return Auton.getCharge();
        }

}
