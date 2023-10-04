package com.frc.robot.utility;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TagManager extends SubsystemBase {
  AprilTagDetector tagDetector;
  AprilTagDetection tagDetection;


  PoseStrategy poseStrategy;

  PhotonCamera photonCamera1;
  PhotonCamera photonCamera2;


  ArrayList<Pair<PhotonCamera, Transform3d>> camspos;
  public Pose2d lastPose = new Pose2d();


  static final Transform3d campos1 = new Transform3d(new Translation3d(18 * (0.0254), 4 * (0.0254), 0),
          new Rotation3d(0, 0.262, 0));
  static final Transform3d campos2 = new Transform3d(new Translation3d(-5.5 * (0.0254), -3 * (0.0254), 0),
          new Rotation3d(0, 0, Math.PI));

  static Path aprilPath = Path.of(Filesystem.getDeployDirectory().getAbsolutePath(),
          "apriltaglayout.json");
  private static AprilTagFieldLayout tagLayout;

  static {
    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      System.err.println("Input sucks L + Ratio");
    }
  }

  public PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(tagLayout, poseStrategy, photonCamera1, campos1);
  public PhotonPoseEstimator poseEstimator2 = new PhotonPoseEstimator(tagLayout, poseStrategy, photonCamera2, campos2);


  public void init() {

    photonCamera1 = new PhotonCamera("Global_Shutter_Camera");

    camspos = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camspos.add(new Pair<>(photonCamera1, campos1));
    camspos.add(new Pair<>(photonCamera2, campos2));

    poseStrategy = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP;
    poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);


  }

  public void detectTag() {
    Pose3d pos = tagLayout.getTags().get(1).pose;

  }

  public boolean detect() {
    Optional<EstimatedRobotPose> result = poseEstimator.update();

    if (result.isEmpty()) {
      return false;
    } else {
      return true;
    }
  }


  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    poseEstimator.setReferencePose(lastPose);


    double currentTime = Timer.getFPGATimestamp();
    Optional<EstimatedRobotPose> result1 = poseEstimator.update();
    Optional<EstimatedRobotPose> result2 = poseEstimator2.update();
    // return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(),
    // currentTime - result.get().getSecond());
    if (result1.isEmpty() && result2.isEmpty()) {
      return null;
    }else{
      if(result1.isPresent()){
        return poseEstimator.update();
      }else{
        lastPose = result2.get().estimatedPose.toPose2d();
        return poseEstimator.update();
      }
    }
  }

  public void update(){
    SmartDashboard.putNumber("Position X: ", poseEstimator.update().get().estimatedPose.getX());
    SmartDashboard.putNumber("Postion Y: ", poseEstimator.update().get().estimatedPose.getY());
  }

  public Command print() {
    return this.run(() -> update());

  }

}