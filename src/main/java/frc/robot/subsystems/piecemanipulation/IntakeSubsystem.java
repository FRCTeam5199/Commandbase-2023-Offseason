// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.piecemanipulation;

import java.util.function.BooleanSupplier;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motorcontrol.SparkMaxController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeSubsystem extends SubsystemBase {
    /** Creates a new BottomIntakeSubsystem. */
    public SparkMaxController bottomIntake;
    public SparkMaxController topLeftIntake;
    public SparkMaxController topRightIntake;

    public DoubleSolenoid bottomPiston;

    public Boolean coneMode;
    public Boolean deployedBottomIntake;

    public IntakeSubsystem() {}

    @Override
    public void periodic() {
      
      // This method will be called once per scheduler run
    }
    public void init() {
      bottomPiston = new DoubleSolenoid(Constants.Pneumatics.PCM_ID,Constants.Pneumatics.PNEUMATICS_MODULE_TYPE, Constants.MotorIDs.SPIKE_OUT_ID, Constants.MotorIDs.SPIKE_IN_ID);
      bottomIntake = new SparkMaxController(Constants.MotorIDs.BottomIntakeMotor_ID, MotorType.kBrushed);
      bottomIntake.setBrake(true);


    }

    ////////////

    public boolean deployedIntakeSwitch(boolean out) {
      return deployedBottomIntake = out;
    }

    public boolean coneCubeMode() {
      return !coneMode;
    }
   
    public CommandBase switchCubeCone() {
      return this.run(()-> coneCubeMode());
    }

    public boolean getConeMode() {
      return coneMode;
    }
    public boolean getDeployedBottomIntake() {
      return deployedBottomIntake;
    }

    ///////////
    
    public CommandBase deployPiston() {
      // return this.run(()-> System.out.println("DEPLOYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY"));
      return this.runOnce(() -> bottomPiston.set(Value.kForward));
      
    }

    public CommandBase retractPiston() {
      // return this.run(()-> System.out.println("RETRACTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT"));
      return this.runOnce(() -> bottomPiston.set(Value.kReverse));
    }

    ////////////

    public Command spinBottomIntake() {
      return this.runOnce(() -> bottomIntake.setPercent(-.6));
    }

    public Command stopSpinBottomIntake() {
      return this.runOnce(() -> bottomIntake.setPercent(0));
    }

    public void spinBothIntakes(){
      spinBottomIntake();
    }

    
}
