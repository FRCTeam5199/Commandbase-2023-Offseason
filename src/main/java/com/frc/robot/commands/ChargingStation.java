package com.frc.robot.commands;

    import com.frc.robot.subsystems.Drivetrain;
    import edu.wpi.first.math.kinematics.ChassisSpeeds;
    import edu.wpi.first.wpilibj.DriverStation;
    import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChargingStation extends CommandBase {

  private final Drivetrain drivetrain;
  private double pitchOffsetDegrees;
  private double rollOffsetDegrees;
  private static final double k = 0.035 / 1.3;

  private boolean rolledOver = true;

  /** Creates a new ChargingStationAuto. */
  public ChargingStation(Drivetrain drivetrain, double pitchOffsetDegrees, double rollOffsetDegrees) {

    addRequirements(drivetrain);
    this.drivetrain = drivetrain;

    this.pitchOffsetDegrees = pitchOffsetDegrees;
    this.rollOffsetDegrees = rollOffsetDegrees;
  }

  public ChargingStation(Drivetrain drivetrain) {
    this(drivetrain, drivetrain.getPitch().getDegrees(), drivetrain.getRoll().getDegrees());
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double speedMultiplier = 1;

    boolean roll = Math.hypot(drivetrain.getPitch().getDegrees() - pitchOffsetDegrees, drivetrain.getRoll().getDegrees() - rollOffsetDegrees) > 1.5;

    if(!rolledOver){
      speedMultiplier = 2;

      if(!roll){
        rolledOver = true;
      }
    } 

    double speed_x = 0.0;
    double speed_y = 0.0;
    if (roll) {
      speed_x = (drivetrain.getRoll().getDegrees() - rollOffsetDegrees) * k * speedMultiplier;
      speed_y = (drivetrain.getPitch().getDegrees() - pitchOffsetDegrees) * k * speedMultiplier;
    }

    drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speed_x,
            speed_y,
            0,
            drivetrain.getGyroscopeRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return !DriverStation.isAutonomousEnabled();
  }
}

