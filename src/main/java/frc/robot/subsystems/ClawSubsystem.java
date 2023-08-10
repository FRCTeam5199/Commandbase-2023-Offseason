package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  private DoubleSolenoid clawPiston;
  private Compressor compressor;

  /** Creates a new Claw. */
  public ClawSubsystem() {}
  /**
   * Example command factory method.
   *
   * @return a command
   */
  // public CommandBase exampleMethodCommand() {
  //   // Inline construction of command goes here.
  //   // Subsystem::RunOnce implicitly requires `this` subsystem.
  //   return runOnce(
  //       () -> {
  //         /* one-time action goes here */
  //       });
  // }

  // /**
  //  * An example method querying a boolean state of the subsystem (for example, a digital sensor).
  //  *
  //  * @return value of some boolean subsystem state, such as a digital sensor.
  //  */
  // public boolean exampleCondition() {
  //   // Query some boolean state, such as a digital sensor.
  //   return false;
  // }

  public void init() {
    System.out.println("Claw - init()");
    pneumaticsInit();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void pneumaticsInit() {
    System.out.println("Claw - pneumaticsInit()");
    compressor = new Compressor(Constants.Pneumatics.PNEUMATICS_MODULE_TYPE);
    compressor.enableDigital();

    clawPiston = new DoubleSolenoid(Constants.Pneumatics.PNEUMATICS_MODULE_TYPE, Constants.Pneumatics.CLAW_IN_ID, Constants.Pneumatics.CLAW_OUT_ID);
  }

  /**
   * Opens the Claw piston
   */
  public CommandBase openPiston() {
    System.out.println("Claw - Opening Piston...");
    return this.runOnce(() -> clawPiston.set(DoubleSolenoid.Value.kForward));
  }

  /**
   * Closes the Claw piston
   */
  public CommandBase closePiston() {
    System.out.println("Claw - Closing Piston...");
    return this.runOnce(() -> clawPiston.set(DoubleSolenoid.Value.kReverse));
  }
}