// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frc.robot.subsystems.piecemanipulation;

import java.util.function.BooleanSupplier;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.frc.robot.Constants;
import com.frc.robot.AbstractMotorInterfaces.SparkMotorController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeSubsystem extends SubsystemBase {
    /** Creates a new BottomIntakeSubsystem. */
    public SparkMotorController bottomIntake;
    public SparkMotorController topLeftIntake;
    public SparkMotorController topRightIntake;

    public DoubleSolenoid bottomPiston;

    public Boolean coneMode;
    public Boolean deployedBottomIntake;
    public Boolean stopBottomIntake;
    public Boolean spinWithCube;
    public Boolean OnceTimer;
    public Boolean biggerIfTimer;

    public Timer bottomIntakeTimer;
    public Timer spinBottomToKeep;
    public Timer checkTheTimer;
  

    public IntakeSubsystem() {}

    @Override
    public void periodic() {
      if (stopBottomIntake == false) {
        runBottomIntake();
      }
      runTheKeeper();
      // This method will be called once per scheduler run
    }
    public void init() {
      bottomIntakeTimer = new Timer();
      spinBottomToKeep = new Timer();
      checkTheTimer = new Timer();
      spinBottomToKeep.start();
      bottomIntakeTimer.start();
      checkTheTimer.start();

      stopBottomIntake = true;
      OnceTimer = false;
      spinWithCube = false;
      biggerIfTimer = true;


      bottomPiston = new DoubleSolenoid(Constants.PCM_ID,Constants.PNEUMATICS_MODULE_TYPE, Constants.SPIKE_OUT_ID, Constants.SPIKE_IN_ID);
      bottomIntakeMotorInit();
      bottomIntake.setBrake(true);
      bottomIntake.setCurrentLimit(10);
      
      

    }

    // public boolean getBottomIntakeStop() {
    //   return stopBottomIntake;

    // }

    public void bottomIntakeMotorInit() {
      if (Constants.RobotNum == 5199) {
        bottomIntake = new SparkMotorController(Constants.BottomIntakeMotor_ID, MotorType.kBrushed);
      }
      // if (Constants.RobotNum == 9199) {
      //   bottomIntake = new Victor
      // }
    }

    ////////////
    public void TimerReset() {
      if (OnceTimer) {
        bottomIntakeTimer.restart();
        OnceTimer = false;
        checkTheTimer.restart();
            }
    }
    
    public void runBottomIntake() {
        if(bottomIntake.getCurrent() < 13.5) {
          bottomIntake.moveAtPercent(-1);
          if  (biggerIfTimer) {
            OnceTimer = true;
            biggerIfTimer = false;
          }
          if (checkTheTimer.get() > .60 && checkTheTimer.get() <0.80){
              biggerIfTimer = true;
          }
            }
        else{
          TimerReset(); 
            stopper();
      }
    }
    
    public void stopper() {
      if (bottomIntakeTimer.get() > 0.5) {
      bottomPiston.set(Value.kReverse);
      bottomIntake.moveAtPercent(0);
      bottomIntakeTimer.reset();
      spinWithCube = true;
      biggerIfTimer = true;
      stopBottomIntake = true;
      
      }
    }

    public void runTheKeeper() {
      if(spinWithCube) {
        spinToKeepIn();
      }
    }

    ///////////
    public void spinToKeepIn() {
        if (spinBottomToKeep.get() > 2){
          bottomIntake.moveAtPercent(-1);
          if(bottomIntake.getCurrent() < 14.5) {
            bottomIntake.moveAtPercent(0);
            spinBottomToKeep.restart();
          }
        }
      }
    
    public CommandBase deployPiston() {
      // return this.run(()-> System.out.println("DEPLOYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY"));
      return this.runOnce(() -> bottomPiston.set(Value.kForward));
      
    }

    public CommandBase retractPiston() {
      // return this.run(()-> System.out.println("RETRACTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT"));
      return this.runOnce(() -> bottomPiston.set(Value.kReverse));
    }

    ////////////

    public void spinBottomOutake(boolean stop) {
      spinWithCube = false;
      if(stop) {
        bottomIntake.moveAtPercent(0);
      }
      else{
        bottomIntake.moveAtPercent(10);
      }
    } 


    public void runTheIntakeWithLimit(){
      bottomPiston.set(Value.kForward);
      stopBottomIntake = false;
    }

    public Command spinBottomWithLimit() {
      return this.runOnce(() -> runTheIntakeWithLimit());
    }

    /**
     * The command spin the intake to spit out cubes
     * @param stop if stop is true it stops intake, if false the intake is free to spin
     * 
     */
    public Command spinOutakeOnBottom(boolean stop) {
      return runOnce(()-> spinBottomOutake(stop));
    }


}