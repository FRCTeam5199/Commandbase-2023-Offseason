package com.frc.robot.commands;

import com.frc.robot.subsystems.piecemanipulation.ArmSubsystem;
import com.frc.robot.subsystems.piecemanipulation.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CompositeCommand extends CommandBase {
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;

    public CompositeCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        elevator = elevatorSubsystem;
        arm = armSubsystem;
        addRequirements(elevator, arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        arm.runOnce(() -> arm.setExtendSetpoint(0)).andThen(() -> elevator.setSetpoint(0)).andThen(arm.setRotateSetpoint(50)).andThen(arm.setExtendSetpoint(10));
        // elevator.runOnce();
        // arm.runOnce(() -> arm.setRotateSetpoint(50));
        // arm.runOnce(() -> arm.setExtendSetpoint(10));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

    
}
