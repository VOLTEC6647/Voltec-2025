
package com.team6647.frc2025.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

import com.team1678.frc2024.RobotState;
import com.team1678.frc2024.RobotState.VisionUpdate;
import com.team1678.frc2024.subsystems.Climber;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.Subsystem;
import com.team6647.frc2025.Robot;
import com.team6647.frc2025.Constants.VisionPhotonConstants;
import com.team6647.frc2025.subsystems.vision.LimelightHelpers.PoseEstimate;
import com.team6647.frc2025.subsystems.vision.LimelightHelpers.RawFiducial;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase{
    private String[] limelights = new String[] {"limelight-coral", "limelight-source"};
    private String[] photons = new String[] {"corall", "coralr"};
    private Transform3d photonTransform[] = new Transform3d[] {VisionPhotonConstants.CAMERA_CORALL_TRANSFORM, VisionPhotonConstants.CAMERA_CORALR_TRANSFORM};
    private ArrayList<GlobalCamera> cameras = new ArrayList<GlobalCamera>();
    private Pose2d bestPose = null;

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
    }

    @Override
    public void periodic() {
        for(int i = 0; i < cameras.size(); i++) {
            GlobalCamera camera = cameras.get(i);
            camera.updateEstimatedPose();
            if(camera.getEstimatedPose()!= null){
            Logger.recordOutput("/Cameras/"+ camera.getName() +"Pose", camera.getEstimatedPose());
            }
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
            bestPose = cameras.get(bestCameraIndex).getEstimatedPose();
            //RobotContainer.instance.drivetrain.addVisionMeasurement(bestPose.pose, bestPose.timestampSeconds);
            RobotState.getInstance().addVisionUpdate(
                new VisionUpdate(
                    cameras.get(bestCameraIndex).getTimestampSeconds(),
                    new com.team254.lib.geometry.Translation2d(cameras.get(bestCameraIndex).getEstimatedPose().getTranslation()),
                    new com.team254.lib.geometry.Translation2d(0,0),//mConstants.kRobotToCamera.getTranslation(), // Use the correct camera offset
                    0.02
                )
            );
        }
    }

    public Pose2d getBestPose(){
        return bestPose;
    }
}