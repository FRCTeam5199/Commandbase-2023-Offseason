package com.frc.robot.subsystems.piecemanipulation;

import com.frc.robot.CompConstants;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  private DoubleSolenoid clawPiston;
  private Compressor compressor;

  public ClawSubsystem() {}

  public void init() {
    // System.out.println("Claw - init()");
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
    // System.out.println("Claw - pneumaticsInit()");

    clawPiston = new DoubleSolenoid(CompConstants.PCM_ID, CompConstants.PNEUMATICS_MODULE_TYPE, CompConstants.Piecemanipulation.CLAW_IN_ID, CompConstants.Piecemanipulation.CLAW_OUT_ID);
  }

  /**
   * Opens the Claw piston
   */
  public Command openPiston() {
    // System.out.println("Claw - Opening Piston...");
    return this.runOnce(() -> clawPiston.set(DoubleSolenoid.Value.kForward));
  }

  /**
   * Closes the Claw piston
   */
  public Command closePiston() {
    // System.out.println("Claw - Closing Piston...");
    return this.runOnce(() -> clawPiston.set(DoubleSolenoid.Value.kReverse));
  }
}