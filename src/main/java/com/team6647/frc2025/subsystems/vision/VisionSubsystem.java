
package com.team6647.frc2025.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

import java.util.ArrayList;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.RobotState.VisionObservation;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

import com.team6647.frc2025.Constants.VisionPhotonConstants;
import com.team6647.lib.util.QuestNav;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionSubsystem extends SubsystemBase{
    private String[] limelights = new String[] {"limelight-coral"};//, "limelight-source"
    private String[] photons = new String[] {};//"corall", "coralr"
    private Transform3d photonTransform[] = new Transform3d[] {VisionPhotonConstants.CAMERA_CORALL_TRANSFORM, VisionPhotonConstants.CAMERA_CORALR_TRANSFORM};
    private ArrayList<GlobalCamera> cameras = new ArrayList<GlobalCamera>();
    @Getter private Pose2d bestPose = null;
    private boolean questNavEnabled = false;
    private GlobalCamera questNavCamera;

    private static VisionSubsystem mInstance;
    
    public static VisionSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new VisionSubsystem();//, CoralPivotConstants.kHoodEncoderConstants
		}
		return mInstance;
	}
    public VisionSubsystem() {
        for(int i = 0; i < limelights.length; i++) {
            String limelight = limelights[i];
            LimelightPoseEstimator limelightPoseEstimator = new LimelightPoseEstimator(limelight);
            cameras.add(new GlobalCamera(limelight, limelightPoseEstimator));
        }

        for(int i = 0; i < photons.length; i++) {
            String photon = photons[i];
            PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2025Reefscape.loadAprilTagLayoutField(), new PhotonCamera(photon), photonTransform[i], VisionPhotonConstants.AMBIGUITY_THRESHOLD, VisionPhotonConstants.TAG_AREA_THRESHOLD);
            cameras.add(new GlobalCamera(photon, photonPoseEstimator));
        }

        if(questNavEnabled) {
            questNavCamera = new GlobalCamera("QuestNav", new QuestNav());
            cameras.add(questNavCamera);
        }
    }

    @Override
    public void periodic() {
        for(int i = 0; i < cameras.size(); i++) {
            GlobalCamera camera = cameras.get(i);
            camera.updateEstimatedPose();
            Logger.recordOutput("/Cameras/" + camera.getName() +"/Area", camera.getTagArea());
            Logger.recordOutput("/Cameras/" + camera.getName() +"/Timestamp", camera.getTimestampSeconds());
            Logger.recordOutput("/Cameras/" + camera.getName() +"/isConnected", camera.isConnected());
            Logger.recordOutput("/Cameras/" + camera.getName() +"/FPS", camera.getFPS());
            if(camera.getEstimatedPose()!= null){
                Logger.recordOutput("/Cameras/" + camera.getName() +"/Pose", camera.getEstimatedPose());
            }
        }
        if(questNavCamera != null){
            Logger.recordOutput("/Cameras/" + questNavCamera.getName() +"/BatteryPercent", questNavCamera.getBatteryPercent());
            Logger.recordOutput("/Cameras/" + questNavCamera.getName() +"/IsTracking", questNavCamera.getTrackingStatus());
            Logger.recordOutput("/Cameras/" + questNavCamera.getName() +"/LostCount", questNavCamera.getTrackingLostCounter());
        }

        int bestCameraIndex = -1;
        double bestCameraArea = 0;

        for(int i = 0; i < cameras.size(); i++) {
            if(cameras.get(i).getTagArea() > bestCameraArea) {
                bestCameraIndex = i;
                bestCameraArea = cameras.get(i).getTagArea();
            }
        }

        if(bestCameraIndex != -1) {
            GlobalCamera bestCamera = cameras.get(bestCameraIndex);
            bestPose = bestCamera.getEstimatedPose();
            //RobotContainer.instance.drivetrain.addVisionMeasurement(bestPose.pose, bestPose.timestampSeconds);
            RobotState.getInstance().addVisionObservation(
                new VisionObservation(
                    bestCamera.getEstimatedPose(),
                    bestCamera.getTimestampSeconds(),
                    VecBuilder.fill(bestCamera.getStdevsXY(), bestCamera.getStdevsXY(), bestCamera.getStdevsRot())
                )
            );
        }
    }
}