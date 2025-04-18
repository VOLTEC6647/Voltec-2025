package com.team6647.frc2025.subsystems.vision;

import org.littletonrobotics.frc2025.RobotState;

import com.team6647.frc2025.Robot;
import com.team6647.frc2025.subsystems.vision.LimelightHelpers.PoseEstimate;
import com.team6647.frc2025.subsystems.vision.PhotonPoseEstimator.EstimatedPose2d;
import com.team6647.lib.util.QuestNav;

import edu.wpi.first.math.geometry.Pose2d;
import lombok.Getter;

public class GlobalCamera {
    private CameraType cameraType;
    private LimelightPoseEstimator limelightPoseEstimator;
    private PhotonPoseEstimator photonPoseEstimator;
    private QuestNav questNav;
    private final String cameraName;
    private Pose2d estimatedPose;
    private double tagArea;
    private double timestampSeconds;
    private boolean consideringAmbiguity = true;
    @Getter private double stdevsXY;
    @Getter private double stdevsRot = Double.POSITIVE_INFINITY;

    public enum CameraType {
        LIMELIGHT,
        PHOTON,
        QUESTNAV
    }

    public GlobalCamera(String name, LimelightPoseEstimator poseEstimator) {
        this.cameraType = CameraType.LIMELIGHT;
        this.cameraName = name;
        this.limelightPoseEstimator = poseEstimator;
        this.stdevsXY = 0.02;
    }

    public GlobalCamera(String name, PhotonPoseEstimator poseEstimator) {
        this.cameraType = CameraType.PHOTON;
        this.cameraName = name;
        this.photonPoseEstimator = poseEstimator;
        this.stdevsXY = 0.02;
    }

    public GlobalCamera(String name, QuestNav questNav) {
        this.cameraType = CameraType.QUESTNAV;
        this.cameraName = name;
        this.questNav = questNav;
        this.stdevsXY = 0.2;
        this.consideringAmbiguity = false;
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
            EstimatedPose2d estimate = photonPoseEstimator.getFilteredEstimatedPose(RobotState.getInstance().getEstimatedPose());
            if (estimate != null && filterAmbiguity(estimate.estimatedPose.getRotation().getDegrees())) {
                this.estimatedPose = estimate.estimatedPose;
                this.tagArea = photonPoseEstimator.poseTagArea;
                this.timestampSeconds = estimate.timestampSeconds;
            } else {
                this.tagArea = 0;  
            }
        } else if (cameraType == CameraType.QUESTNAV) {
            Pose2d pose = questNav.getPose();
            this.estimatedPose = pose;
            this.tagArea = Double.POSITIVE_INFINITY;
            this.timestampSeconds = questNav.timestamp();
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
            if(Math.abs(rotation-RobotState.getInstance().getHeading().getDegrees())>6){
                return false;
            }
        }
        return true;
    }

    public void setConsideringAmbiguity(boolean value){
        consideringAmbiguity = value;
    }

    public boolean isConnected() {
        if (cameraType == CameraType.LIMELIGHT) {
            return limelightPoseEstimator.isLimelightConnected();
        } else if (cameraType == CameraType.PHOTON) {
            return photonPoseEstimator.isConnected();
        } else if (cameraType == CameraType.QUESTNAV) {
            return questNav.connected();
        }
        return false;
    }

    public double getFPS() {
        if (cameraType == CameraType.LIMELIGHT) {
            return limelightPoseEstimator.getFPS();
        } else if (cameraType == CameraType.PHOTON) {
            return photonPoseEstimator.getFPS();
        } else if (cameraType == CameraType.QUESTNAV) {
            return questNav.getFPS();
        }
        return 0;
    }
    
}
