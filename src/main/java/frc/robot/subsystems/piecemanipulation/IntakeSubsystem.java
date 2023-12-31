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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeSubsystem extends SubsystemBase {
    /** Creates a new BottomIntakeSubsystem. */
    public SparkMaxController bottomIntake;
    public SparkMaxController topLeftIntake;
    public SparkMaxController topRightIntake;

    public DoubleSolenoid bottomPiston;

    public Boolean coneMode;
    public Boolean deployedBottomIntake;
    public Boolean stopBottomIntake;

    public Timer bottomIntakeTimer;

    public IntakeSubsystem() {}

    @Override
    public void periodic() {
      
      // This method will be called once per scheduler run
    }
    public void init() {
      bottomIntakeTimer = new Timer();
      stopBottomIntake = false;
      bottomPiston = new DoubleSolenoid(Constants.Pneumatics.PCM_ID,Constants.Pneumatics.PNEUMATICS_MODULE_TYPE, Constants.MotorIDs.SPIKE_OUT_ID, Constants.MotorIDs.SPIKE_IN_ID);
      bottomIntakeMotorInit();
      bottomIntake.setBrake(true);
      bottomIntake.setCurrentLimit(10);


    }

    // public boolean getBottomIntakeStop() {
    //   return stopBottomIntake;

    // }

    public void bottomIntakeMotorInit() {
      if (Constants.RobotNum == 5199) {
        bottomIntake = new SparkMaxController(Constants.MotorIDs.BottomIntakeMotor_ID, MotorType.kBrushed);
      }
      // if (Constants.RobotNum == 9199) {
      //   bottomIntake = new Victor
      // }
    }

    ////////////

    // public boolean deployedIntakeSwitch(boolean out) {
    //   return deployedBottomIntake = out;
    // }

    // public boolean coneCubeMode() {
    //   return !coneMode;
    // }
   
    // public CommandBase switchCubeCone() {
    //   return this.run(()-> coneCubeMode());
    // }

    // public boolean getConeMode() {
    //   return coneMode;
    // }
      public Command changeIntakeStop() {
          return runOnce(()-> stopBottomIntake = false);
        }
      

    public boolean getDeployedBottomIntake() {
      return deployedBottomIntake;
    }

    
    public void runBottomIntake() {
      bottomIntakeTimer.start();
      if(stopBottomIntake == false){
        if(bottomIntake.getCurrent() < 16.5) {
          bottomIntake.setPercent(-1);
          // System.out.println("-----------------------------------------------------------------------------------------");
          }
      else{
        // bottomIntake.setPercent(0);
        if  (bottomIntakeTimer.get() > 2) {
        bottomPiston.set(Value.kReverse);
        }
        bottomIntakeTimer.reset();
        // System.out.println("stopppppppppppppppppppppppppppppppppppppppppppppppppppppppppppppp");
        stopBottomIntake = true;
      }
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

    public void spinBottomOutake(boolean stop) {
      if(stop) {
        bottomIntake.setPercent(0);
      }
      else{
        bottomIntake.setPercent(10);
      }
    } 


    public Command spinBottomWithLimit() {
      return this.run(() -> runBottomIntake());
    }

    /**
     * The command spin the intake to spit out cubes
     * @param stop if stop is true it stops intake, if false the intake is free to spin
     * 
     */
    public Command spinOutakeOnBottom(boolean stop) {
      return runOnce(()-> spinBottomOutake(stop));
    }

    public Command victorRunBottomIntake() {
      return runOnce(()-> bottomIntake.setPercent(-1));
    }

    public Command StopBottomIntake() {
      bottomIntakeTimer.reset();
      return runOnce(()-> bottomIntake.setPercent(0));
    }

}
