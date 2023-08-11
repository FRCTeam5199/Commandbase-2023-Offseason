package swervelib.encoders;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.jni.ConfigJNI;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.spns.SpnValue;
import com.ctre.phoenixpro.hardware.core.*;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

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
  
    factoryDefault();
    clearStickyFaults();
    configure(false);


    coder.getConfigurator().apply(canConfiguration);
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
    coder.getMagnetHealth().refresh();
    StatusSignal<MagnetHealthValue> magnetstrength = coder.getMagnetHealth();


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
    return coder.getAbsolutePosition().refresh().getValue();

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
    coder.getConfigurator().apply(new CANcoderConfiguration());
  }
}
