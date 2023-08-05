package swervelib.encoders;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.MagnetSensorConfigs;
import com.ctre.phoenixpro.configs.jni.ConfigJNI;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.configs.CANcoderConfigurator;
import com.ctre.phoenixpro.hardware.DeviceIdentifier;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.MagnetHealthValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;
import com.ctre.phoenixpro.spns.SpnValue;
import com.ctre.phoenixpro.hardware.core.*;
import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.wpilibj.DriverStation;


/**
 * Swerve Absolute Encoder for CTRE CANCoders.
 */
public class CANCoderSwerve extends SwerveAbsoluteEncoder
{

  /**
   * CANCoder with WPILib sendable and support.
   */
  public CANcoder coder;
  public CANcoderConfiguration canConfiguration = new CANcoderConfiguration();
  public CANcoderConfigurator canConfigurator;
  public DeviceIdentifier idfier;


  /**
   * Initialize the CANCoder on the standard CANBus.
   *
   * @param id CAN ID.
   */
  public CANCoderSwerve(int id)
  {
    coder = new CANcoder(id);
  }

  /**
   * Initialize the CANCoder on the CANivore.
   *
   * @param id     CAN ID.
   * @param canbus CAN bus to initialize it on.
   */
  public CANCoderSwerve(int id, String canbus)
  {
    coder = new CANcoder(id, canbus);
    idfier = new DeviceIdentifier(id, "CANCoder", canbus);
    canConfigurator = new CANcoderConfigurator(idfier);
  
    factoryDefault();
    clearStickyFaults();
    configure(true);

    canConfigurator.refresh(canConfiguration);
  }


  /**
   * Reset the encoder to factory defaults.
   */
  /**
   * Clear sticky faults on the encoder.
   */
  @Override
  public void clearStickyFaults()
  {
    coder.clearStickyFaults();
  }

  /**
   * Configure the absolute encoder to read from [0, 360) per second.
   *
   * @param inverted Whether the encoder is inverted.
   */
  @Override
  public void configure(boolean inverted)
  {

    canConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    canConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canConfiguration.MagnetSensor.MagnetOffset = 0;

    canConfigurator.apply(canConfiguration);

  }

  /**
   * Get the absolute position of the encoder. Sets {@link SwerveAbsoluteEncoder#readingError} on erroneous readings.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  @Override
  public double getAbsolutePosition()
  {
    readingError = false;
    StatusSignalValue<MagnetHealthValue> magnetstrength = coder.getMagnetHealth();


    if (magnetstrength.getValue() != MagnetHealthValue.Magnet_Green){
      DriverStation.reportWarning(
          "CANCoder " + coder.getDeviceID() + " magnetic field is less than ideal.\n", false);
    }
    if (magnetstrength.getValue() == MagnetHealthValue.Magnet_Invalid)
    {
      readingError = true;
      DriverStation.reportWarning("CANCoder " + coder.getDeviceID() + " reading was faulty.\n", false);
      return 0;
    }

    return coder.getAbsolutePosition().getValue();

    // Taken from democat's library.
    // Source: https://github.com/democat3457/swerve-lib/blob/7c03126b8c22f23a501b2c2742f9d173a5bcbc40/src/main/java/com/swervedrivespecialties/swervelib/ctre/CanCoderFactoryBuilder.java#L51-L74
  }


  /**
   * Get the instantiated absolute encoder Object.
   *
   * @return Absolute encoder object.
   */
  @Override
  public Object getAbsoluteEncoder()
  {
    return coder;
  }

  @Override
  public void factoryDefault() {
    canConfigurator.apply(new CANcoderConfiguration());
  }
}
