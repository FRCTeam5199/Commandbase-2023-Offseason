// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frc.robot.controls;

import static com.frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;

import com.frc.robot.Constants;
import com.frc.robot.commands.DriveCommand;
import com.frc.robot.utility.NumberStepper;
import com.frc.robot.utility.PovNumberStepper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ManualControls implements DriveCommand.Controls {
  private XboxController xbox;

  private final PovNumberStepper speed;
  private final PovNumberStepper turnSpeed;

  private NetworkTable limelightTable;

  private NetworkTable climbTable;

  private NetworkTable intakeTable;

  private XboxController controlStation;

  private NetworkTable armTable;

  private boolean prevclimbEngaged;
  private boolean climbEngaged;

  private boolean prevIntakeOverrideEngaged;

  private boolean intakeOverrideEngaged;

  /**
   * Creates a new `ManualControls` instance.
   *
   * @param xbox the Xbox controller to use for manual control
   */
  public ManualControls(XboxController xbox) {
    this(xbox, null);
  }

  public ManualControls(XboxController xbox, XboxController controlStation) {
    this.xbox = xbox;
    if (controlStation == null)
      this.controlStation = xbox;
    else
      this.controlStation = controlStation;

    this.speed = new PovNumberStepper(
        new NumberStepper(Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.4, 0.0,
            Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.6, Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.025),
        xbox,
        PovNumberStepper.PovDirection.VERTICAL);

    this.turnSpeed = new PovNumberStepper(
        new NumberStepper(Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.175, 0.0,
            Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.15,
            Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.0025),
        xbox,
        PovNumberStepper.PovDirection.HORIZONTAL);
    
    ntDispTab("Controls")
      .add("Y Joystick", this::driveY)
      .add("X Joystick", this::driveX)
      .add("Rotation Joystick", this::driveRotationX);

    
    limelightTable = NetworkTableInstance.getDefault().getTable("Limelight");
    intakeTable = NetworkTableInstance.getDefault().getTable("Intake");
    armTable = NetworkTableInstance.getDefault().getTable("Arm");
    intakeTable.getEntry("speed").setDouble(0.0);

    // limelightTable.getEntry("limelightPipeline").setInteger( (long) 0);
    
    climbTable = NetworkTableInstance.getDefault().getTable("Climb");

    climbTable.getEntry("isClimbing").setBoolean(false);

    climbTable.getEntry("climbVelocityL").setDouble(0.0);

    climbTable.getEntry("climbVelocityR").setDouble(0.0);

    armTable.getEntry("resetArmZero").setBoolean(false);

    armTable.getEntry("overrideSoftLimits").setBoolean(false);

  }

  /**
   * Constructs an event instance around the A button's digital signal.
   *
   * @return an event instance representing the A button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger a() {
    return a(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public boolean rB(){
    return xbox.getRightBumper();
  }

  /**
   * Constructs an event instance around the A button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal attached to the given
   *     loop.
   */
  public Trigger a(EventLoop loop) {
    return xbox.a(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around the B button's digital signal.
   *
   * @return an event instance representing the B button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger b() {
    return b(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the B button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the B button's digital signal attached to the given
   *     loop.
   */
  public Trigger b(EventLoop loop) {
    return xbox.b(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around the X button's digital signal.
   *
   * @return an event instance representing the X button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger x() {
    return x(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the X button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the X button's digital signal attached to the given
   *     loop.
   */
  public Trigger x(EventLoop loop) {
    return xbox.x(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around the Y button's digital signal.
   *
   * @return an event instance representing the Y button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger y() {
    return y(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the Y button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Y button's digital signal attached to the given
   *     loop.
   */
  public Trigger y(EventLoop loop) {
    return xbox.y(loop).castTo(Trigger::new);
  }

  public boolean rT() {return xbox.getRightBumper();}

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.01);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  private static double modifyAxis2(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  /**
   * Returns the x component of the robot's velocity, as controlled by the Xbox
   * controller.
   *
   * @return the x component of the robot's velocity
   */
  @Override
  public double driveX() {
    return - modifyAxis(xbox.getLeftY() * speed.get());
  }

  /**
   * Returns the y component of the robot's velocity, as controlled by the Xbox
   * controller.
   *
   * @return the y component of the robot's velocity
   */
  @Override
  public double driveY() {
    return - modifyAxis(xbox.getLeftX() * speed.get());
  }

  /**
   * Returns the angular velocity of the robot, as controlled by the Xbox
   * controller.
   *
   * @return the angular velocity of the robot
   */
  @Override
  public double driveRotationX() {
    return modifyAxis(-xbox.getRightX() * turnSpeed.get());
  }

  @Override
  public double driveRotationY() {
    return modifyAxis(-xbox.getRightY() * turnSpeed.get());
  }

  /**
   * Returns whether the yaw of the robot's gyroscope should be reset, as
   * controlled by the Xbox controller.
   *
   * @return whether the yaw of the robot's gyroscope should be reset
   */
  @Override
  public boolean driveResetYaw() {
    return xbox.getStartButton();
  }


  @Override
  public boolean driveResetGlobalPose() {
    return xbox.getBackButton();
  }

  }


