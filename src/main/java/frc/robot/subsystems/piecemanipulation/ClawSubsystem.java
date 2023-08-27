package frc.robot.subsystems.piecemanipulation;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  private DoubleSolenoid clawPiston;
  private Compressor compressor;

  public ClawSubsystem() {}

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
    // compressor = new Compressor(Constants.Pneumatics.PNEUMATICS_MODULE_TYPE);
    // compressor.enableDigital();

    clawPiston = new DoubleSolenoid(Constants.Pneumatics.PCM_ID,Constants.Pneumatics.PNEUMATICS_MODULE_TYPE, Constants.Pneumatics.CLAW_IN_ID, Constants.Pneumatics.CLAW_OUT_ID);
  }

  /**
   * Opens the Claw piston
   */
  public Command openPiston() {
    System.out.println("Claw - Opening Piston...");
    return this.runOnce(() -> clawPiston.set(DoubleSolenoid.Value.kForward));
  }

  /**
   * Closes the Claw piston
   */
  public Command closePiston() {
    System.out.println("Claw - Closing Piston...");
    return this.runOnce(() -> clawPiston.set(DoubleSolenoid.Value.kReverse));
  }
}