// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.auto.Autos;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import java.io.File;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.CloseAction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/falcon"));

  final ClawSubsystem claw = new ClawSubsystem();
  
  final ElevatorSubsystem elevator = new ElevatorSubsystem();
  
  final ArmSubsystem arm = new ArmSubsystem();
  
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed


  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  final IntakeSubsystem intake = new IntakeSubsystem();

  XboxController driverXbox = new XboxController(0);

  CommandXboxController commandXboxController = new CommandXboxController(1);
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    arm.init();
    elevator.init();
    claw.init();

    // Configure the trigger bindings
    configureBindings();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
                                                          // Applies deadbands and inverts controls because joysticks
                                                          // are back-right positive while robot
                                                          // controls are front-left positive
                                                          () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                        Constants.OperatorConstants.LEFT_Y_DEADBAND),
                                                          () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                        Constants.OperatorConstants.LEFT_X_DEADBAND),
                                                          () -> -driverXbox.getRightX(),
                                                          () -> -driverXbox.getRightY(),
                                                          false);

    AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
                                                                         () ->
                                                                             MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                             Constants.OperatorConstants.LEFT_Y_DEADBAND),
                                                                         () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                         Constants.OperatorConstants.LEFT_X_DEADBAND),
                                                                         () -> driverXbox.getRawAxis(2), false);
    TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
                                                    () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                 Constants.OperatorConstants.LEFT_Y_DEADBAND),
                                                    () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                    Constants.OperatorConstants.LEFT_X_DEADBAND),
                                                    () -> driverXbox.getRawAxis(4), () -> true, true, false);
    TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRawAxis(4), () -> true, false, true);

    drivebase.setDefaultCommand(simClosedFieldRel);
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    System.out.println("RobotContainer - configureBindings()");
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

//     new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
//     new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
// //    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

    commandXboxController.leftBumper().onTrue(arm.resetEncoder());

    commandXboxController.leftBumper().onTrue(elevator.resetEncoder());

    commandXboxController.povRight().onTrue(claw.openPiston());
    commandXboxController.povLeft().onTrue(claw.closePiston());

	if (Constants.PieceManipulation.ARM_ELEVATOR_MANUAL) {
		commandXboxController.y().whileTrue(elevator.moveElevator(1));
		commandXboxController.a().whileTrue(elevator.moveElevator(-1));
	}

	if (Constants.PieceManipulation.ARM_ELEVATOR_MANUAL) {
		commandXboxController.povUp().whileTrue(arm.moveArm(-1));
		commandXboxController.povDown().whileTrue(arm.moveArm(1));
	}

    // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

  
  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return Autos.Autons(drivebase,"path");
  } 

  public void setDriveMode() {}

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
