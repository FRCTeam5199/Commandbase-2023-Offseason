// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.*;


public class CompressorSubsystem extends SubsystemBase {
  public PneumaticHub pneumaticsHub;

  
  /** Creates a new Compressor. */
  public CompressorSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void init() {
    if (Constants.Pneumatics.PNEUMATICS_MODULE_TYPE == PneumaticsModuleType.REVPH) {
      pneumaticsHub = new PneumaticHub(Constants.Pneumatics.PCM_ID);
      pneumaticsHub.clearStickyFaults();
    }
  
  }

  public CommandBase runCompression() {
    return run(() ->pneumaticsHub.enableCompressorDigital());
  }

}
