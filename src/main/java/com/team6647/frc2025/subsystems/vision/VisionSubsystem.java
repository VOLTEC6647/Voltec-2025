
package com.team6647.frc2025.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import lombok.Getter;

import java.util.ArrayList;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.RobotState.VisionObservation;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.Utils;
import com.team4678.CommandSwerveDrivetrain;
import com.team6647.frc2025.Constants.VisionPhotonConstants;
import com.team6647.lib.util.QuestNav;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionSubsystem extends SubsystemBase{
    private String[] limelights = new String[] {"limelight-coral"};//, "limelight-source"
    private String[] photons = new String[] {};//"corall", "coralr"
    private Transform3d photonTransform[] = new Transform3d[] {VisionPhotonConstants.CAMERA_CORALL_TRANSFORM, VisionPhotonConstants.CAMERA_CORALR_TRANSFORM};
    private ArrayList<GlobalCamera> cameras = new ArrayList<GlobalCamera>();
    @Getter public Pose2d bestPose = new Pose2d();
    private boolean questNavEnabled = false;
    private GlobalCamera questNavCamera;
    QuestNav questNav = null;
    public GlobalCamera coralLimelight;
    public boolean hasFirstPose = false;
    public boolean positionreset = true;

    private static VisionSubsystem mInstance;
    
    public static VisionSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new VisionSubsystem();
		}
		return mInstance;
	}
    public VisionSubsystem() {
        for(int i = 0; i < limelights.length; i++) {
            String limelight = limelights[i];
            LimelightPoseEstimator limelightPoseEstimator = new LimelightPoseEstimator(limelight);
            GlobalCamera toAdd = new GlobalCamera(limelight, limelightPoseEstimator);
            cameras.add(toAdd);
            if(toAdd.getName()=="limelight-coral"){
                coralLimelight = toAdd;
            }
        }

        for(int i = 0; i < photons.length; i++) {
            String photon = photons[i];
            PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField(), new PhotonCamera(photon), photonTransform[i], VisionPhotonConstants.AMBIGUITY_THRESHOLD, VisionPhotonConstants.TAG_AREA_THRESHOLD);
            cameras.add(new GlobalCamera(photon, photonPoseEstimator));
        }

        if(questNavEnabled) {
            questNav = QuestNav.getInstance();
            questNavCamera = new GlobalCamera("QuestNav", questNav);
            cameras.add(questNavCamera);
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput("/Cameras/bestPose", bestPose);
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
                Logger.recordOutput("/Cameras/bestCameraTimestamp", cameras.get(bestCameraIndex).getTimestampSeconds());
            }
        }
        Logger.recordOutput("/Cameras/bestCameraIndex", bestCameraIndex);

        if(bestCameraIndex != -1) {
            GlobalCamera bestCamera = cameras.get(bestCameraIndex);
            Logger.recordOutput("/Cameras/bestCamera", bestCamera.getName());
 
            CommandSwerveDrivetrain.getInstance().addVisionMeasurement(
                    new Pose2d(
                        bestCamera.getEstimatedPose().getX(),
                        bestCamera.getEstimatedPose().getY(),
                        CommandSwerveDrivetrain.getInstance().getHeading()
                    ),
                    Utils.fpgaToCurrentTime(bestCamera.getTimestampSeconds()),
                    VecBuilder.fill(bestCamera.getStdevsXY(), bestCamera.getStdevsXY(), 999999)
            );
        }
    }

    public void setQuestPose(Pose2d pose) {
        if(questNavCamera != null) {
            questNavCamera.setPosition(pose);
        }
    }
}