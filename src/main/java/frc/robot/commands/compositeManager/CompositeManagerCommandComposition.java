package frc.robot.commands.compositeManager;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.piecemanipulation.ArmSubsystem;
import frc.robot.subsystems.piecemanipulation.ElevatorSubsystem;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class CompositeManagerCommandComposition extends SequentialCommandGroup {
  /**
   * Creates a new ComplexAuto.
   *
   * @param drive The drive subsystem this command will run on
   * @param hatch The hatch subsystem this command will run on
   */ 
  public CompositeManagerCommandComposition(ArmSubsystem arm, ElevatorSubsystem elevator) {
    addCommands();
  }
}