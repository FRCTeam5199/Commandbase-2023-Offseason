// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.compositeManager.CompositeManagerCommandComposition;
import frc.robot.commands.swervedrive.auto.Autos;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.controllers.CommandButtonPanel;
import frc.robot.misc.AprilTagManager;
import frc.robot.subsystems.piecemanipulation.ArmSubsystem;
import frc.robot.subsystems.piecemanipulation.ClawSubsystem;
import frc.robot.subsystems.piecemanipulation.ElevatorSubsystem;
import frc.robot.subsystems.piecemanipulation.IntakeSubsystem;
import frc.robot.subsystems.piecemanipulation.WristSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.Constants.Drivebase;
import frc.robot.commands.CompressorCommand;
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

  public static final ArmSubsystem arm = new ArmSubsystem();
  
  public static final ElevatorSubsystem elevator = new ElevatorSubsystem();
  
  public static final WristSubsystem wrist = new WristSubsystem();
  
  public static final ClawSubsystem claw = new ClawSubsystem();

  public static final IntakeSubsystem intake = new IntakeSubsystem();

  public final CompressorSubsystem compressor = new CompressorSubsystem();
  

  final AprilTagManager tagManager = new AprilTagManager();

  // final SendableChooser<Command> sendableChooser = new SendableChooser<>();
  
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed


  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
//   final IntakeSubsystem intake = new IntakeSubsystem();

  public static final CommandButtonPanel commandButtonPanel = new CommandButtonPanel(Constants.BUTTON_PANEL1, Constants.BUTTON_PANEL2);

  XboxController driverXbox = new XboxController(0);

  CommandXboxController commandXboxController = new CommandXboxController(1);

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    compressor.init();

    claw.init();
    
    elevator.init();
    
    arm.init();
    
    wrist.init();

    intake.init();

    tagManager.init();
    
    /*private*/ final Command compositeManagerCommandComposition = new CompositeManagerCommandComposition(arm, elevator);

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
        () -> MathUtil.applyDeadband(driverXbox.getRawAxis(4), .8), () -> true, true, false);

    TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRawAxis(4), () -> true, false, true);

    CompressorCommand compressorRun = new CompressorCommand(compressor);

    drivebase.setDefaultCommand(simClosedFieldRel);
    compressor.setDefaultCommand(compressorRun);
    
    
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

    // commandXboxController.leftBumper().onTrue(arm.resetRotateEncoder());
    // commandXboxController.leftBumper().onTrue(arm.resetExtendEncoder());

    // commandXboxController.leftBumper().onTrue(elevator.resetEncoder());
    
    // commandXboxController.leftBumper().onTrue(wrist.resetEncoder());


    if (Constants.PieceManipulation.ENABLE_INTAKE && Constants.PieceManipulation.INTAKE_MANUAL) {
      commandXboxController.y().onTrue(intake.deployPiston());
      commandXboxController.a().onTrue(intake.retractPiston());

    }

    commandButtonPanel.button(3, 10).onTrue(intake.deployPiston());
    commandButtonPanel.button(3, 9).onTrue(intake.deployPiston());
    if (Constants.PieceManipulation.ENABLE_CLAW) {
      // commandXboxController.a().onTrue(claw.openPiston());
      // commandXboxController.y().onTrue(claw.closePiston());
    }
    if (Constants.PieceManipulation.ARM_ELEVATOR_MANUAL && Constants.PieceManipulation.ENABLE_ELEVATOR) {
      // commandXboxController.x().whileTrue(elevator.move(1));
      // commandXboxController.b().whileTrue(elevator.move(-1));
    }
    // TEMPORARY ELSE STATEMENT REMOVE LATER
    else {
      // commandXboxController.x().whileTrue(elevator.setSetpoint(5));
      // commandXboxController.b().whileTrue(elevator.setSetpoint(30));
    }

    if (Constants.PieceManipulation.ARM_ELEVATOR_MANUAL && Constants.PieceManipulation.ENABLE_ARM) {
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

    if (Constants.PieceManipulation.WRIST_MANUAL && Constants.PieceManipulation.ENABLE_WRIST) {
      commandXboxController.leftBumper().whileTrue(wrist.move(0.5f));
      commandXboxController.rightBumper().whileTrue(wrist.move(-0.5f));
    }
    if (Constants.PieceManipulation.INTAKE_MANUAL && Constants.PieceManipulation.ENABLE_INTAKE) {

      // commandXboxController.x().onTrue(intake.spinBottomWithLimit(false));
      // commandXboxController.x().onFalse(intake.spinBottomWithLimit(true));
      if(Constants.RobotNum == 5199) {
        commandXboxController.x().onTrue(intake.spinBottomWithLimit(false));
        commandXboxController.x().onFalse(intake.spinBottomWithLimit(true));
        commandXboxController.b().onTrue(intake.spinOutakeOnBottom(false));
        commandXboxController.b().onFalse(intake.spinOutakeOnBottom(true));
      }
      else {
        // commandXboxController.x().onTrue(intake.spinBottomIntake());
      // commandXboxController.x().onFalse(intake.stopSpinBottomIntake());
      }
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
