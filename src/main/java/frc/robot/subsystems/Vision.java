// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstruction;

public class Vision extends SubsystemBase {
  private PhotonCamera cam = new PhotonCamera("OV5647");
  private PhotonPoseEstimator photonPoseEstimator;
  /** Creates a new Vision. */
  public Vision() {
    try {
      // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
      AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile);
      // Create pose estimator
      photonPoseEstimator =
              new PhotonPoseEstimator(
                      fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, cam, RobotConstruction.robotToCam);
    } catch (IOException e) {
      // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
      // where the tags are.
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
    }//Stolen from photon lib example
  }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
     *     the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
      if (photonPoseEstimator == null) {
          // The field layout failed to load, so we cannot estimate poses.
          return Optional.empty();
      }
      photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      return photonPoseEstimator.update();
    }//Stolen from photonlib example code

    public PhotonPipelineResult getResults(int pipe){
      cam.setDriverMode(false);
      cam.setPipelineIndex(pipe);
      return cam.getLatestResult();
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
