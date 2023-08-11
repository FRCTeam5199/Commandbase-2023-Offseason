package swervelib.simulation.ctre;

import static swervelib.simulation.ctre.PhysicsSim.random;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;


import swervelib.simulation.ctre.PhysicsSim.SimProfile;

/**
 * Holds information about a simulated TalonFX.
 */
class TalonFXSimProfile extends SimProfile
{

  private final TalonFX _falcon;
  private final TalonFXSimState _falconsim;
  private final double  _accelToFullTime;
  private final double  _fullVel;
  private final boolean _sensorPhase;

  /** The current position */
  // private double _pos = 0;
  /**
   * The current velocity
   */
  private double _vel = 0;

  /**
   * Creates a new simulation profile for a TalonFX device.
   *
   * @param falcon          The TalonFX device
   * @param accelToFullTime The time the motor takes to accelerate from 0 to full, in seconds
   * @param fullVel         The maximum motor velocity, in ticks per 100ms
   * @param sensorPhase     The phase of the TalonFX sensors
   */
  public TalonFXSimProfile(
      final TalonFX falcon,
      TalonFXSimState falconsim,
      final double accelToFullTime,
      final double fullVel,
      final boolean sensorPhase)
  {
    this._falcon = falcon;
    this._falconsim = falconsim;
    this._accelToFullTime = accelToFullTime;
    this._fullVel = fullVel;
    this._sensorPhase = sensorPhase;
    falconsim = falcon.getSimState();
  
  }

  /**
   * Runs the simulation profile.
   *
   * <p>This uses very rudimentary physics simulation and exists to allow users to test features of
   * our products in simulation using our examples out of the box. Users may modify this to utilize more accurate
   * physics simulation.
   */
  public void run()
  {
    final double period      = getPeriod();
    final double accelAmount = _fullVel / _accelToFullTime * period / 1000;

    /// DEVICE SPEED SIMULATION

    double outPerc =  _falconsim.getMotorVoltage()/ 12;
    if (_sensorPhase)
    {
      outPerc *= -1;
    }
    // Calculate theoretical velocity with some randomness
    double theoreticalVel = outPerc * _fullVel * random(0.95, 1);
    // Simulate motor load
    if (theoreticalVel > _vel + accelAmount)
    {
      _vel += accelAmount;
    } else if (theoreticalVel < _vel - accelAmount)
    {
      _vel -= accelAmount;
    } else
    { 
      _falcon.getPosition();
      _falcon.getVelocity();
      _vel += 0.9 * (theoreticalVel - _vel);

      double supplyCurrent = Math.abs(outPerc) * 30 * random(0.95, 1.05);
      double statorCurrent = outPerc == 0 ? 0 : supplyCurrent / Math.abs(outPerc);

      _falconsim.setSupplyVoltage(statorCurrent);

    }
    // _pos += _vel * period / 100;

    /// SET SIM PHYSICS INPUTS

  }
}
