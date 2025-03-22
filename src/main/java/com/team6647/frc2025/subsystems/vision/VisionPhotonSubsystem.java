
package com.team6647.frc2025.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionPhotonSubsystem extends SubsystemBase{
    private final PhotonPoseEstimator cameraPoseEstimatorSource;
    private final PhotonCamera sourceCamera;

    private static VisionPhotonSubsystem mInstance;
    
    public static VisionPhotonSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new VisionPhotonSubsystem();//, CoralPivotConstants.kHoodEncoderConstants
		}
		return mInstance;
	}
    public VisionPhotonSubsystem() {
        sourceCamera = new PhotonCamera("corall");
        cameraPoseEstimatorSource = new PhotonPoseEstimator(AprilTagFields.k2025Reefscape.loadAprilTagLayoutField(), sourceCamera, VisionPhotonConstants.CAMERA_SOURCE_TRANSFORM, VisionPhotonConstants.AMBIGUITY_THRESHOLD, VisionPhotonConstants.TAG_AREA_THRESHOLD);
    }

    @Override
    public void periodic() {
        Optional<PhotonPoseEstimator.EstimatedPose3d> estimatedCameraSourcePose = cameraPoseEstimatorSource.getEstimatedPose(RobotState.getInstance().getLatestFieldToVehicle().toLegacy());
        SmartDashboard.putBoolean("hasPresentPhoton", estimatedCameraSourcePose.isPresent());
            if (estimatedCameraSourcePose.isPresent()) {
                // update pose estimator using front limelight
                Logger.recordOutput("/Auto/PhotonPoseOffset", estimatedCameraSourcePose.get().estimatedPose.toPose2d());
                
                RobotState.getInstance().addVisionUpdate(
                new VisionUpdate(
                    estimatedCameraSourcePose.get().timestampSeconds,
                    new com.team254.lib.geometry.Translation2d(estimatedCameraSourcePose.get().estimatedPose.getTranslation().toTranslation2d()),
                    new com.team254.lib.geometry.Translation2d(0,0),//mConstants.kRobotToCamera.getTranslation(), // Use the correct camera offset
                    0.02
                )   
            );
            
            }
    }
}