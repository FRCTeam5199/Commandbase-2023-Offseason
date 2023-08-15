package frc.robot.commands.swervedrive.apriltags;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.misc.AprilTagManager;

public class AprilCommand extends CommandBase {
    private final AprilTagManager tagManager;

    public AprilCommand(AprilTagManager tagManager){
        this.tagManager = tagManager;
    }

    @Override
    public void initialize(){
        tagManager.init();
    }

    @Override
    public void execute(){
        tagManager.print();
    }

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
