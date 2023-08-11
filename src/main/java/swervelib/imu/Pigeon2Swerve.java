package swervelib.imu;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;



/**
 * SwerveIMU interface for the Pigeon2
 */
public class Pigeon2Swerve extends SwerveIMU
{

  /**
   * Pigeon2 IMU device.
   */
  Pigeon2 imu;
  /**
   * Offset for the Pigeon 2.
   */

  private Rotation3d offset = new Rotation3d();

  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid  CAN ID for the pigeon
   * @param canbus CAN Bus name the pigeon resides on.
   */
  public Pigeon2Swerve(int canid, String canbus)
  {
    imu = new Pigeon2(canid, canbus);
    Pigeon2Configuration config = new Pigeon2Configuration();
    factoryDefault();
    config.FutureProofConfigs = true;
    config.Pigeon2Features.EnableCompass = true;
    config.Pigeon2Features.DisableNoMotionCalibration = true;
    config.Pigeon2Features.DisableTemperatureCompensation = true;
    config.GyroTrim.GyroScalarX = 0;
    config.GyroTrim.GyroScalarY = 0;
    config.GyroTrim.GyroScalarZ = 0;
    config.MountPose.MountPosePitch = 0;
    config.MountPose.MountPoseRoll = 0;
    config.MountPose.MountPoseYaw = 0;
    imu.getConfigurator().apply(config);
    SmartDashboard.putData(imu);
  }

  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid CAN ID for the pigeon
   */
  public Pigeon2Swerve(int canid)
  {
    this(canid, "");
  }

  /**
   * Reset IMU to factory default.
   */
  @Override
  public void factoryDefault()
  {
    imu.getConfigurator().apply(new Pigeon2Configuration());
    Pigeon2FeaturesConfigs imuUtilConfig = new Pigeon2FeaturesConfigs();
  
    imuUtilConfig.EnableCompass = true; // Compass utilization causes readings to jump dramatically in some cases.
  }

  /**
   * Clear sticky faults on IMU.
   */
  @Override
  public void clearStickyFaults()
  {
    imu.clearStickyFaults();
  }

  /**
   * Set the gyro offset.
   *
   * @param offset gyro offset as a {@link Rotation3d}.
   */
  public void setOffset(Rotation3d offset)
  {
    this.offset = offset;
  }

  /**
   * Fetch the {@link Rotation3d} from the IMU without any zeroing. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  @Override
  public Rotation3d getRawRotation3d()
  {
    double w;
    double x;
    double y;
    double z;
    w = imu.getQuatW().refresh().getValue();
    x = imu.getQuatX().refresh().getValue();
    y = imu.getQuatY().refresh().getValue();
    z = imu.getQuatZ().refresh().getValue();
        return new Rotation3d(new Quaternion(w, x, y, z));
  }

  /**
   * Fetch the {@link Rotation3d} from the IMU. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  @Override
  public Rotation3d getRotation3d()
  {
    return getRawRotation3d().minus(offset);
  }

  /**
   * Fetch the acceleration [x, y, z] from the IMU in meters per second squared. If acceleration isn't supported returns
   * empty.
   *
   * @return {@link Translation3d} of the acceleration as an {@link Optional}.
   */
  @Override
  public Optional<Translation3d> getAccel()
  {
    double x;
    double y;
    double z;
    x = imu.getAccelerationX().refresh().getValue();
    y = imu.getAccelerationY().refresh().getValue();
    z = imu.getAccelerationZ().refresh().getValue();
    return Optional.of(new Translation3d(x, y, z).times(9.81 / 16384.0));
  }

  /**
   * Get the instantiated IMU object.
   *
   * @return IMU object.
   */
  @Override
  public Object getIMU()
  {
    return imu;
  }
}
