package com.team6647.frc2025.subsystems.vision;

import com.team1678.frc2024.RobotState;
import com.team1678.frc2024.subsystems.Drive;
import com.team6647.frc2025.Robot;
import com.team6647.frc2025.subsystems.vision.LimelightHelpers.PoseEstimate;
import com.team6647.frc2025.subsystems.vision.PhotonPoseEstimator.EstimatedPose2d;

import edu.wpi.first.math.geometry.Pose2d;

public class GlobalCamera {
    private CameraType cameraType;
    private LimelightPoseEstimator limelightPoseEstimator;
    private PhotonPoseEstimator photonPoseEstimator;
    private final String cameraName;
    private Pose2d estimatedPose;
    private double tagArea;
    private double timestampSeconds;
    private boolean consideringAmbiguity = true;

    public enum CameraType {
        LIMELIGHT,
        PHOTON
    }

    public GlobalCamera(String name, LimelightPoseEstimator poseEstimator) {
        this.cameraType = CameraType.LIMELIGHT;
        this.cameraName = name;
        this.limelightPoseEstimator = poseEstimator;
    }

    public GlobalCamera(String name, PhotonPoseEstimator poseEstimator) {
        this.cameraType = CameraType.PHOTON;
        this.cameraName = name;
        this.photonPoseEstimator = poseEstimator;
    }

    public void updateEstimatedPose() {
        if (cameraType == CameraType.LIMELIGHT) {
            PoseEstimate estimate = limelightPoseEstimator.getEstimatedPose();
            if(estimate != null && filterAmbiguity(estimate.pose.getRotation().getDegrees())) {
                this.estimatedPose = estimate.pose;
                this.tagArea = estimate.avgTagArea;
                this.timestampSeconds = estimate.timestampSeconds;
            }else{
                this.tagArea = 0;
            }

        } else if (cameraType == CameraType.PHOTON) {
            EstimatedPose2d estimate = photonPoseEstimator.getFilteredEstimatedPose(RobotState.getInstance().getLatestFieldToVehicle().toLegacy());
            if (estimate != null && filterAmbiguity(estimate.estimatedPose.getRotation().getDegrees())) {
                this.estimatedPose = estimate.estimatedPose;
                this.tagArea = photonPoseEstimator.poseTagArea;
                this.timestampSeconds = estimate.timestampSeconds;
            } else {
                this.tagArea = 0;  
            }
        }
    }

    public double getTagArea() {
        return tagArea;
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public double getTimestampSeconds() {
        return timestampSeconds;
    }

    public String getName() {
        return cameraName;
    }

    public boolean filterAmbiguity(double rotation) {
        if(consideringAmbiguity && Robot.is_event){
            if(Math.abs(rotation-Drive.getInstance().getHeading().getDegrees())>6){
                return false;
            }
        }
        return true;
    }

    public void setConsideringAmbiguity(boolean value){
        consideringAmbiguity = value;
    }
    
}
