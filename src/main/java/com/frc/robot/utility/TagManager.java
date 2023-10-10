package com.frc.robot.utility;

import java.io.IOException;
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
import edu.wpi.first.wpilibj.DriverStation;
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

  PhotonCamera photonCamera;
  PhotonCamera photonCamera2;
  PhotonPoseEstimator poseEstimator;
  PhotonPoseEstimator poseEstimator2;
  public TagManager(){

    photonCamera = new PhotonCamera("Global_Shutter_Camera");
    photonCamera2 = new PhotonCamera("HD_USB_Camera");
    try {
      // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
      AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      // Create pose estimator
      System.out.println("created");
      poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, new Transform3d(new Translation3d(18 * (0.0254), 4 * (0.0254), 0), new Rotation3d(0, 0.262, 0))); 
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      poseEstimator2 = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera2, new Transform3d(new Translation3d(-5.5 * (0.0254), -3 * (0.0254), 0), new Rotation3d(0, 0, Math.PI)));
      poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    } catch (IOException e) {
      // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
      // where the tags are.
      System.out.println("Not working");
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      poseEstimator = null;
    }
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
  
      var result = photonCamera.getLatestResult();
      var result2 = photonCamera2.getLatestResult();
      poseEstimator.setReferencePose(prevEstimatedRobotPose);
      poseEstimator2.setReferencePose(prevEstimatedRobotPose);
      if(result.hasTargets()){
        return poseEstimator.update();
      }else if(result2.hasTargets()){
        return poseEstimator2.update();
      }else{
        return Optional.empty();
      }
  }
}
