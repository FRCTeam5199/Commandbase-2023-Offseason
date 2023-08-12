package frc.robot.commands.piecemanipulation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.piecemanipulation.WristSubsystem;

public class Wrist extends CommandBase{
    
    // Called every time the scheduler runs while the command is scheduled.
        @Override
    public void execute()
    {
        double armRot = ArmSubsystem.armRotateMotor.getRotations();
        double wristRot = WristSubsystem.wrist.getRotations();
        if(armRot >= -125){
            if(wristRot >= 0 && wristRot <= 4000){
                WristSubsystem.wrist.setVolts(0);
            }else{
                WristSubsystem.wrist.setVolts(-6);
            }
        }else {
            if(wristRot <= 4011 && wristRot >= 10){
                WristSubsystem.wrist.setVolts(0);
            }else{
                WristSubsystem.wrist.setVolts(6);
            }
        }

    }


        @Override
    public void initialize()
    {
    }

    // Called once the command ends or is interrupted.
        @Override
    public void end(boolean interrupted)
    {

    }

    // Returns true when the command should end.
        @Override
    public boolean isFinished()
    {
      return false;
    }
}
