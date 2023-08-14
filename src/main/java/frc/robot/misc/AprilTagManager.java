package frc.robot.misc;

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
import edu.wpi.first.wpilibj.Timer;

import org.photonvision.PhotonPoseEstimator.*;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.Timer.*;


public class AprilTagManager {
    AprilTagDetector tagDetector;
    AprilTagDetection tagDetection;

    AprilTagPoseEstimate poseEstimate;
    PhotonPoseEstimator poseEstimator;

    PoseStrategy poseStrategy;

    PhotonCamera photonCamera1;
    PhotonCamera photonCamera2;

    Pose2d lastPose = new Pose2d(new Translation2d(0,0),new Rotation2d(0));

    ArrayList<Pair<PhotonCamera, Transform3d>> camspos;

    static final Transform3d campos1 = new Transform3d(new Translation3d(18*(0.0254), 4*(0.0254), 0), new Rotation3d(0, 0.262, 0));
    static final Transform3d camposBack = new Transform3d(new Translation3d(-5.5*(0.0254), -3*(0.0254), 0), new Rotation3d(0, 0, Math.PI));



    static Path aprilPath = Path.of(Filesystem.getDeployDirectory().getAbsolutePath(), "apriltags", "2023-chargedup.json");
    private static AprilTagFieldLayout tagLayout;
  static{
    try{
      tagLayout = new AprilTagFieldLayout(aprilPath);
    }catch(Exception e){
      System.err.println("Input sucks L + Ratio");
    }
  }

  public void init(){

    photonCamera1 = new PhotonCamera("Global_Shutter_Camera");
    photonCamera2 = new PhotonCamera("HD_USB_CAMERA");
    camspos = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camspos.add(new Pair<>(photonCamera1, campos1));
     camspos.add(new Pair<>(photonCamera2, camposBack));

    poseStrategy = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP;
    poseEstimator = new PhotonPoseEstimator(tagLayout, poseStrategy, camspos.get(0).getFirst(), camspos.get(0).getSecond());
    poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
}


    public void detectTag(){
        Pose3d pos = tagLayout.getTags().get(1).pose;

    }

    public Pair<Pose2d, Double> getEstimatedGlobalPose()
    {
      poseEstimator.setReferencePose(lastPose);

      double currentTime = Timer.getFPGATimestamp();
      Optional<EstimatedRobotPose> result = poseEstimator.update();

      if (result.isEmpty()) {
          return new Pair<Pose2d, Double>(new Pose2d(-2,-2,new Rotation2d(0)), 0.0);
      } else {
          //return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());

          lastPose = result.get().estimatedPose.toPose2d();
          return new Pair<Pose2d, Double>(lastPose, 0.0);
      }

    }

    public void updateGeneric()
    {
      UserInterface.smartDashboardPutNumber("Apriltag Estimate X Pose: ",getEstimatedGlobalPose().getFirst().getX());
      UserInterface.smartDashboardPutNumber("Apriltag Estimate Y Pose: ",getEstimatedGlobalPose().getFirst().getY());
    }

}
