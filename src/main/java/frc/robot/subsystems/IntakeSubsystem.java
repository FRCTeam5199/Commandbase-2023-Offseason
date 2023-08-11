// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motorcontrol.SparkMaxController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
      bottomPiston = new DoubleSolenoid(Constants.Pneumatics.PNEUMATICS_MODULE_TYPE, Constants.MotorIDs.SPIKE_OUT_ID, Constants.MotorIDs.SPIKE_IN_ID);
      bottomIntake = new SparkMaxController(Constants.MotorIDs.BottomIntakeMotor_ID);
      bottomIntake.setBrake(true);

      topLeftIntake = new SparkMaxController(Constants.MotorIDs.TopIntakeLeft_ID);
      topRightIntake = new SparkMaxController(Constants.MotorIDs.TopIntakeRigth_ID);
      topLeftIntake.setBrake(true);
      topRightIntake.setBrake(true);
      topLeftIntake.setCurrentLimit(20);
      topLeftIntake.setCurrentLimit(20);

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
      System.out.println("-------------------------deploy Piston-------------------------------------");
      if (getConeMode() == false) {
      deployedIntakeSwitch(true);
      return this.runOnce(() -> bottomPiston.set(Value.kForward));
      }
      return null;
    }

    public CommandBase retractPiston() {
      System.out.println("-------------------------retract piston-------------------------------------");
      deployedIntakeSwitch(false);
      return this.runOnce(() -> bottomPiston.set(Value.kReverse));
    }

    ////////////

    public void spinBottomIntake() {
      bottomIntake.setPercent(-.6);
    }

    public void TopIntake() {
      topLeftIntake.setPercent(12);
      topRightIntake.setPercent(-12);
    }
    public void spinBothIntakes(){
      spinBottomIntake();
      TopIntake();
    }

    public CommandBase runIntakes() {
      System.out.println("-------------------------running intake-------------------------------------");
      if (getConeMode()) {
        return this.run(()->TopIntake());
      }
      else if (!getConeMode() && getDeployedBottomIntake()) {
        return this.run(() -> spinBothIntakes());
      }
      return null;
    }

    
}
