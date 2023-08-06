package swervelib.motors;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import com.ctre.phoenixpro.configs.*;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.controls.*;
import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.hardware.DeviceIdentifier;

import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.sim.TalonFXSimState;
import com.ctre.phoenixpro.hardware.DeviceIdentifier;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.math.SwerveMath;
import swervelib.parser.PIDFConfig;
import swervelib.simulation.ctre.PhysicsSim;
import swervelib.telemetry.SwerveDriveTelemetry;


/**
 * {@link com.ctre.phoenix.motorcontrol.can.TalonFX} Swerve Motor. Made by Team 1466 WebbRobotics.
 */
public class TalonFXSwerve extends SwerveMotor
{

  DeviceIdentifier deviceIdentifier;
  /**
   * Factory default already occurred.
   */
  private final boolean              factoryDefaultOccurred = false;
  /**
   * Current TalonFX configuration.
   */




  /**
   * Whether the absolute encoder is integrated.
   */
  private final boolean              absoluteEncoder        = false;
  /**
   * TalonFX motor controller.
   */
  TalonFX motor;

  private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
  private final TalonFXConfigurator talonconfigurator;
  /**
   * The position conversion factor to convert raw sensor units to Meters Per 100ms, or Ticks to Degrees.
   */
  private double  positionConversionFactor = 1;
  /**
   * If the TalonFX configuration has changed.
   */
  private boolean configChanged            = true;
  /**
   * Nominal voltage default to use with feedforward.
   */
  private double  nominalVoltage           = 12.0;

  NeutralOut nOut = new NeutralOut();


  /**
   * Constructor for TalonFX swerve motor.
   *
   * @param motor        Motor to use.
   * @param isDriveMotor Whether this motor is a drive motor.
   */
  public TalonFXSwerve(TalonFX motor, boolean isDriveMotor)
  {
    this.isDriveMotor = isDriveMotor;
    this.motor = motor;
    TalonFXSimState talonsim = motor.getSimState();
    

    factoryDefaults();
    clearStickyFaults();

    deviceIdentifier = new DeviceIdentifier(motor.getDeviceID(), motor.getDescription(), motor.getCANBus());

    talonconfigurator = new TalonFXConfigurator(deviceIdentifier);

    talonConfigure();
    
    talonconfigurator.apply(talonConfig);
    
    motor.getConfigurator();
    if (SwerveDriveTelemetry.isSimulation)
    {
      PhysicsSim.getInstance().addTalonFX(motor, talonsim, .25, 6800);
    }
  }


  /**
   * Construct the TalonFX swerve motor given the ID and CANBus.
   *
   * @param id           ID of the TalonFX on the CANBus.
   * @param canbus       CANBus on which the TalonFX is on.
   * @param isDriveMotor Whether the motor is a drive or steering motor.
   */
  public TalonFXSwerve(int id, String canbus, boolean isDriveMotor)
  {
    this(new TalonFX(id, canbus), isDriveMotor);

  }



  /**
   * Construct the TalonFX swerve motor given the ID.
   *
   * @param id           ID of the TalonFX on the canbus.
   * @param isDriveMotor Whether the motor is a drive or steering motor.
   */
  public TalonFXSwerve(int id, boolean isDriveMotor)
  {
    this(new TalonFX(id), isDriveMotor);

  }

  /**
   * Configure the factory defaults.
   */
  @Override
  
  public void factoryDefaults(){

    if (!factoryDefaultOccurred)
    {
      motor.getConfigurator().apply(new TalonFXConfiguration());
    }
  }

  public void talonConfigure(){
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    talonConfig.MotorOutput.DutyCycleNeutralDeadband = .01;
    talonConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .25;
    talonConfig.Feedback.FeedbackRotorOffset = 0;
    talonConfig.Feedback.FeedbackRemoteSensorID = 1;
    talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    talonConfig.FutureProofConfigs = true;
    talonConfig.Audio.BeepOnBoot = true;

    talonConfig.Feedback.FeedbackRemoteSensorID = motor.getDeviceID();
    talonConfig.Feedback.RotorToSensorRatio = 12.8;
    talonConfig.Feedback.SensorToMechanismRatio = 1;

    talonconfigurator.apply(talonConfig);

  }

  /**
   * Clear the sticky faults on the motor controller.
   */
  @Override
  public void clearStickyFaults()
  {
    motor.clearStickyFaults();
  }

  /**
   * Set the absolute encoder to be a compatible absolute encoder.
   *
   * @param encoder The encoder to use.
   */
  @Override
  public SwerveMotor setAbsoluteEncoder(SwerveAbsoluteEncoder encoder)
  {
    // Do not support.
    return this;
  }

  /**
   * Configure the integrated encoder for the swerve module. Sets the conversion factors for position and velocity.
   *
   * @param positionConversionFactor The conversion factor to apply for position.
   *                                 <p><br>
   *                                 Degrees: <br>
   *                                 <code>
   *                                 360 / (angleGearRatio * encoderTicksPerRotation)
   *                                 </code><br>
   *                                 <p><br>
   *                                 Meters:<br>
   *                                 <code>
   *                                 (Math.PI * wheelDiameter) / (driveGearRatio * encoderTicksPerRotation)
   *                                 </code>
   */
  @Override
  public void configureIntegratedEncoder(double positionConversionFactor)
  {
    this.positionConversionFactor = positionConversionFactor;
    // Taken from democat's library.
    // https://github.com/democat3457/swerve-lib/blob/7c03126b8c22f23a501b2c2742f9d173a5bcbc40/src/main/java/com/swervedrivespecialties/swervelib/ctre/Falcon500DriveControllerFactoryBuilder.java#L16
    configureCANStatusFrames(250);
  }

  /**
   * Set the CAN status frames.
   *
   * @param CANStatus1 Applied Motor Output, Fault Information, Limit Switch Information
   */
  public void configureCANStatusFrames(int CANStatus1)
  {
    //Work on Later
  }

  /**
   * Set the CAN status frames.
   *
   * @param CANStatus1       Applied Motor Output, Fault Information, Limit Switch Information
   * @param CANStatus2       Selected Sensor Position (PID 0), Selected Sensor Velocity (PID 0), Brushed Supply Current
   *                         Measurement, Sticky Fault Information
   * @param CANStatus3       Quadrature Information
   * @param CANStatus4       Analog Input, Supply Battery Voltage, Controller Temperature
   * @param CANStatus8       Pulse Width Information
   * @param CANStatus10      Motion Profiling/Motion Magic Information
   * @param CANStatus12      Selected Sensor Position (Aux PID 1), Selected Sensor Velocity (Aux PID 1)
   * @param CANStatus13      PID0 (Primary PID) Information
   * @param CANStatus14      PID1 (Auxiliary PID) Information
   * @param CANStatus21      Integrated Sensor Position (Talon FX), Integrated Sensor Velocity (Talon FX)
   * @param CANStatusCurrent Brushless Supply Current Measurement, Brushless Stator Current Measurement
   */
  public void configureCANStatusFrames(int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4, int CANStatus8,
                                       int CANStatus10, int CANStatus12, int CANStatus13, int CANStatus14,
                                       int CANStatus21, int CANStatusCurrent)
  {
    StatusCode statusCode;
    // TODO: Configure Status Frame 2 thru 21 if necessary
    // https://v5.docs.ctr-electronics.com/en/stable/ch18_CommonAPI.html#setting-status-frame-periods
  }

  /**
   * Configure the PIDF values for the closed loop controller. 0 is disabled or off.
   *
   * @param config Configuration class holding the PIDF values.
   */
  @Override
  public void configurePIDF(PIDFConfig config)
  {
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = config.p;
    slot0.kI = config.i;
    slot0.kD = config.d;
    slot0.kV = config.f;
    slot0.kS = config.iz;
    configChanged = true;

    motor.getConfigurator().apply(slot0, .05);
  }

  /**
   * Configure the PID wrapping for the position closed loop controller.
   *
   * @param minInput Minimum PID input.
   * @param maxInput Maximum PID input.
   */
  @Override
  public void configurePIDWrapping(double minInput, double maxInput)
  {
    // Do nothing
  }

  /**
   * Set the idle mode.
   *
   * @param isBrakeMode Set the brake mode.
   */
  @Override
  public void setMotorBrake(boolean isBrakeMode)
  {
    motor.setControl(nOut);
  }

  /**
   * Set the motor to be inverted.
   *
   * @param inverted State of inversion.
   */
  @Override
  public void setInverted(boolean inverted)
  {
    Timer.delay(1);
    motor.setInverted(inverted);
  }

  /**
   * Save the configurations from flash to EEPROM.
   */
  @Override
  public void burnFlash()
  {
    if (configChanged)
    {
      configChanged = false;
    }
  }

  /**
   * Set the percentage output.
   *
   * @param percentOutput percent out for the motor controller.
   */
  @Override
  public void set(double percentOutput)
  {
    motor.set(percentOutput);
  }

  /**
   * Convert the setpoint into native sensor units.
   *
   * @param setpoint Setpoint to mutate. In meters per second or degrees.
   * @param position Position in degrees, only used on angle motors.
   * @return Setpoint as native sensor units. Encoder ticks per 100ms, or Encoder tick.
   */
  public double convertToNativeSensorUnits(double setpoint, double position)
  {
    setpoint =
        isDriveMotor ? setpoint * .1 : SwerveMath.placeInAppropriate0To360Scope(position, setpoint);
    return setpoint / positionConversionFactor;
  }

  /**
   * Set the closed loop PID controller reference point.
   *
   * @param setpoint    Setpoint in MPS or Angle in degrees.
   * @param feedforward Feedforward in volt-meter-per-second or kV.
   */
  @Override
  public void setReference(double setpoint, double feedforward)
  {
    setReference(setpoint, feedforward, getPosition());
  }

  /**
   * Set the closed loop PID controller reference point.
   *
   * @param setpoint    Setpoint in meters per second or angle in degrees.
   * @param feedforward Feedforward in volt-meter-per-second or kV.
   * @param position    Only used on the angle motor, the position of the motor in degrees.
   */
  @Override
  public void setReference(double setpoint, double feedforward, double position)
  {

    if (SwerveDriveTelemetry.isSimulation)
    {
      PhysicsSim.getInstance().run();
    }

    burnFlash();

    if (isDriveMotor)
    {

      motor.setControl(new VelocityDutyCycle(convertToNativeSensorUnits(setpoint, position),
      true,
      feedforward/nominalVoltage, 
      0, 
      false));

    } else
    {
      motor.setControl(new PositionDutyCycle(convertToNativeSensorUnits(setpoint, position), true, feedforward, 0, false));
    }
  }

  /**
   * Get the velocity of the integrated encoder.
   *
   * @return velocity in Meters Per Second, or Degrees per Second.
   */
  @Override
  public double getVelocity()
  {
    return (motor.getVelocity().getValue() * 10) * positionConversionFactor;
  }

  /**
   * Get the position of the integrated encoder.
   *
   * @return Position in Meters or Degrees.
   */
  @Override
  public double getPosition()
  {
    StatusSignalValue<Double> rotorPosSignal = motor.getRotorPosition();

    double rotorpos = rotorPosSignal.getValue() * positionConversionFactor;

    return rotorpos;
  }

  /**
   * Set the integrated encoder position.
   *
   * @param position Integrated encoder position. Should be angle in degrees or meters.
   */
  @Override
  public void setPosition(double position)
  {

    PositionDutyCycle positionmove;
    if (!absoluteEncoder && !SwerveDriveTelemetry.isSimulation)
    {
      position = position < 0 ? (position % 360) + 360 : position;
      positionmove = new PositionDutyCycle(position, true, .1, 0, false);
      motor.setControl(positionmove);
    }
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   */
  @Override
  public void setVoltageCompensation(double nominalVoltage)
  {
    VoltageConfigs vConfigs = new VoltageConfigs();
    vConfigs.SupplyVoltageTimeConstant = .01;
    vConfigs.PeakForwardVoltage = nominalVoltage;
    vConfigs.PeakReverseVoltage = nominalVoltage;
    configChanged = true;
    this.nominalVoltage = nominalVoltage;
    motor.getConfigurator().apply(vConfigs);
  }

  /**
   * Set the current limit for the swerve drive motor, remember this may cause jumping if used in conjunction with
   * voltage compensation. This is useful to protect the motor from current spikes.
   *
   * @param currentLimit Current limit in AMPS at free speed.
   */
  @Override
  public void setCurrentLimit(int currentLimit)
  {
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = currentLimit;
    currentLimits.SupplyCurrentLimitEnable = true;
    configChanged = true;

    motor.getConfigurator().apply(currentLimits);
  }

  /**
   * Set the maximum rate the open/closed loop output can change by.
   *
   * @param rampRate Time in seconds to go from 0 to full throttle.
   */
  @Override
  public void setLoopRampRate(double rampRate)
  {
    ClosedLoopRampsConfigs closedRamp = new ClosedLoopRampsConfigs();
    OpenLoopRampsConfigs openRamp = new OpenLoopRampsConfigs();
    openRamp.DutyCycleOpenLoopRampPeriod = rampRate;
    closedRamp.DutyCycleClosedLoopRampPeriod = rampRate;
    configChanged = true;

    motor.getConfigurator().apply(closedRamp);
    motor.getConfigurator().apply(openRamp);
  }

  /**
   * Get the motor object from the module.
   *
   * @return Motor object.
   */
  @Override
  public Object getMotor()
  {
    return motor;
  }

  /**
   * Queries whether the absolute encoder is directly attached to the motor controller.
   *
   * @return connected absolute encoder state.
   */
  @Override
  public boolean isAttachedAbsoluteEncoder()
  {
    return absoluteEncoder;
  }
}
