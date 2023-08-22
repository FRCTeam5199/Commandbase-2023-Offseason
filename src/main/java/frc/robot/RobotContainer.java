// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swervedrive.auto.AutoBalanceCommand;
import frc.robot.commands.swervedrive.auto.Autos;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.piecemanipulation.ArmSubsystem;
import frc.robot.subsystems.piecemanipulation.ClawSubsystem;
import frc.robot.subsystems.piecemanipulation.ElevatorSubsystem;
import frc.robot.subsystems.piecemanipulation.WristSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.misc.AprilTagManager;
import frc.robot.commands.swervedrive.auto.AutoBalanceCommand;

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
  
  final WristSubsystem wrist = new WristSubsystem();

  final AprilTagManager tagManager = new AprilTagManager();


  
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed


  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
//   final IntakeSubsystem intake = new IntakeSubsystem();

  XboxController driverXbox = new XboxController(0);

  CommandXboxController commandXboxController = new CommandXboxController(1);
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

	  claw.init();

    elevator.init();
    
	  arm.init();

	  wrist.init();

    tagManager.init();
    // Configure the trigger bindings
    configureBindings();


    tagManager.setDefaultCommand(tagManager.print());



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
                                                                         () -> MathUtil.applyDeadband(driverXbox.getRawAxis(4), 0.5), false);
    TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
                                                    () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                 Constants.OperatorConstants.LEFT_Y_DEADBAND),
                                                    () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                    Constants.OperatorConstants.LEFT_X_DEADBAND),
                                                    () -> MathUtil.applyDeadband(driverXbox.getRawAxis(4), .3), () -> true, false, true);
    TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(4), () -> true, false, false);

    drivebase.setDefaultCommand(closedFieldRel);
    
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
//     new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
//     new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
// //    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

    commandXboxController.leftBumper().onTrue(arm.resetEncoder());
    commandXboxController.leftBumper().onTrue(elevator.resetEncoder());
    commandXboxController.leftBumper().onTrue(wrist.resetEncoder());

    commandXboxController.povRight().onTrue(claw.openPiston());
    commandXboxController.povLeft().onTrue(claw.closePiston());

    if (Constants.PieceManipulation.ARM_ELEVATOR_MANUAL) {
      commandXboxController.y().whileTrue(elevator.moveElevator(1));
      commandXboxController.a().whileTrue(elevator.moveElevator(-1));
    } else {
      commandXboxController.y().whileTrue(elevator.setElevatorSetpoint(20));
      commandXboxController.a().whileTrue(elevator.setElevatorSetpoint(0));
    }

    if (Constants.PieceManipulation.ARM_ELEVATOR_MANUAL) {
      commandXboxController.povUp().whileTrue(arm.moveArm(-1));
      commandXboxController.povDown().whileTrue(arm.moveArm(1));
    } else {
      commandXboxController.povUp().whileTrue(arm.setArmSetpoint(20));
      commandXboxController.povDown().whileTrue(arm.setArmSetpoint(0));
    }

    if (Constants.PieceManipulation.WRIST_MANUAL) {
      commandXboxController.b().whileTrue(wrist.moveWrist(50));
      commandXboxController.x().whileTrue(wrist.moveWrist(-50));
    } else {
      commandXboxController.b().whileTrue(wrist.setWristSetpoint(10));
      commandXboxController.x().whileTrue(wrist.setWristSetpoint(0));
    }
  }
  // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return Autos.Autons(drivebase, null);
  } 

  public void setDriveMode() {}

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
