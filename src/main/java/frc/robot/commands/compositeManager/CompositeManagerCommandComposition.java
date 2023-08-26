package frc.robot.commands.compositeManager;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.piecemanipulation.ArmSubsystem;
import frc.robot.subsystems.piecemanipulation.ElevatorSubsystem;

/** A composite manager command. */
public class CompositeManagerCommandComposition extends SequentialCommandGroup {
  /**
   * Creates a new CompositeManager Command Composition.
   */ 
  public CompositeManagerCommandComposition(ArmSubsystem arm, ElevatorSubsystem elevator) {
    addCommands(
      arm.runEnd(() -> System.out.println("WORKS!^^^^^^^^^^^^^^^"), ()-> System.out.println("end.............")/*() -> arm.rotateMotorController.setPercent(5), () -> arm.rotateMotorController.setPercent(0)*/)
    );
  }
}