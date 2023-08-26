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
      bottomIntakeMotorInit();
      bottomIntake.setBrake(true);


    }

    public void bottomIntakeMotorInit() {
      if (Constants.RobotNum == 5199) {
        bottomIntake = new SparkMaxController(Constants.MotorIDs.BottomIntakeMotor_ID, MotorType.kBrushed);
      }
      // if (Constants.RobotNum == 9199) {
      //   bottomIntake = new Victor
      // }
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

    
    public void runBottomIntake(boolean currentLimit) {
      if (currentLimit == false) {   
        if (bottomIntake.getCurrent() <= 0.70){
          bottomIntake.setPercent(-1);
        }
        else {
          currentLimit = true;         
      }
    }
      else {
        bottomIntake.setPercent(0);
        System.out.println("CURRENT LIMIT REACHEDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");
      }
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

    public void spinBottomIntake(boolean stop) {
      if(stop) {
        bottomIntake.setPercent(0);
      }
      else{
        bottomIntake.setPercent(3);
      }
    } 

    /**
     * The command that runs the bottom intake with stop when the cube is inside the intake
     * @param currentLimit If true stops the intake, if false runs the intake and continuosly checks for current, 
     * if current exceeds a limit currentLimit becomes true and stops the intake
     */
    public Command spinBottomWithLimit(boolean currentLimit) {
      return this.runOnce(() -> runBottomIntake(currentLimit));
    }

    /**
     * The command spin the intake to spit out cubes
     * @param stop if stop is true it stops intake, if false the intake is free to spin
     * 
     */
    public Command spinOutakeOnBottom(boolean stop) {
      return runOnce(()-> spinBottomIntake(stop));
    }

    
}
