
package com.team6647.frc2025.subsystems.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.team1678.frc2024.RobotState;
import com.team1678.frc2024.RobotState.VisionUpdate;
import com.team1678.frc2024.subsystems.Climber;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.Subsystem;
import com.team6647.frc2025.Robot;
import com.team6647.frc2025.subsystems.limelight.LimelightHelpers.PoseEstimate;
import com.team6647.frc2025.subsystems.limelight.LimelightHelpers.RawFiducial;

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

public class VisionSubsystem extends Subsystem{

    public static final String FRONT_LIMELIGHT = "limelight-front";
    public static final String BACK_LIMELIGHT = "limelight";

    // Adjustable transform for the Limelight pose per-alliance
    private static final Transform2d LL_BLUE_TRANSFORM = new Transform2d(0, 0, new Rotation2d());
    private static final Transform2d LL_RED_TRANSFORM = new Transform2d(0, 0, new Rotation2d());

    public AprilTagFieldLayout aprilTagFieldLayout;
    private boolean odometryEnabled = true;
    private double lastOdometryTime = 0;

    private double limelightHeartbeat = 0;
    private double frontLimelightHeartbeat = 0;
    private double lastHeartbeatTime = 0;
    private double frontLastHeartbeatTime = 0;
    public boolean limelightConnected = false;
    private boolean frontLimelightConnected = false;

    private boolean canSeeSpeaker = false;
    private Double speakerDistance = null;
    private Double speakerAngle = null;
    private boolean shootOnTheFlyEnabled = true;
    private boolean isShootingOnTheFly = false;
    private boolean directTagAiming = true;
    public Double distanceOverride = null;
    private Double speakerTagHeight = 0.0;
    private Double tagHeightOverride = null;
    private PoseEstimate lastPoseEstimate = null;
    private Pose2d currentTrapPose = null;
    public Double stageTX = null;
    public Double stageTY = null;
    private boolean megatag2Enabled = false;
    public HashMap<Integer, Pose2d> trapPoses = new HashMap<>();

    private final RawFiducial[] emptyFiducials = new RawFiducial[0];
    public RawFiducial[] rawFiducials = emptyFiducials;
    private final Pose2d nilPose = new Pose2d(-1, -1, new Rotation2d());

    final int[] autoTagFilter = new int[] {1,2,6,7,8,9,10,11,12,13,17,18,19,20,21,22};
    final int[] teleopTagFilter = new int[] {1,2,6,7,8,9,10,11,12,13,17,18,19,20,21,22};

    public PoseEstimate lastPose;

    PoseEstimate bestPose;
    PoseEstimate backPose;
    double newHeartbeat;


    private static VisionSubsystem mInstance;
    
    public static VisionSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new VisionSubsystem();//, CoralPivotConstants.kHoodEncoderConstants
		}
		return mInstance;
	}
    public VisionSubsystem() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
        } catch (Exception e) {
            aprilTagFieldLayout = null;
        }

        SmartDashboard.putNumber("BLUE Distance Offset", 0);
        SmartDashboard.putNumber("RED Distance Offset", 0);

        SmartDashboard.putNumber("RED Pixel Offset", 0);
        SmartDashboard.putNumber("BLUE Pixel Offset", 0);       
    }

    @Override
    public void readPeriodicInputs() {

        //Logger.recordOutput("Odometry Enabled", odometryEnabled);
        
        newHeartbeat = LimelightHelpers.getLimelightNTDouble("limelight", "hb");
        if (newHeartbeat > limelightHeartbeat) {
            limelightConnected = true;
            limelightHeartbeat = newHeartbeat;
            lastHeartbeatTime = Timer.getFPGATimestamp();
        } else if (Timer.getFPGATimestamp() - lastHeartbeatTime >= 1) {
            limelightConnected = false;
        }
        if (Robot.isSimulation()) limelightConnected = true;

        if (!limelightConnected) {
            rawFiducials = emptyFiducials;
            speakerAngle = null;
            speakerDistance = null;
            speakerTagHeight = null;
            //Logger.recordOutput("LL Pose Valid?", false);
            //Logger.recordOutput("LL Pose", nilPose);
            return;
        }


        bestPose = null;
        backPose = null;
        PoseEstimate megaTag2Pose = null;

        
        if (megatag2Enabled) {
            double megatagDegrees = Drive.getInstance().getHeading().getDegrees();
            if (Robot.is_red_alliance) megatagDegrees = MathUtil.inputModulus(megatagDegrees + 180, -180, 180);
            LimelightHelpers.SetRobotOrientation(BACK_LIMELIGHT, megatagDegrees, 0, 0, 0, 0, 0);
            backPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(BACK_LIMELIGHT);
        } else {
            backPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(BACK_LIMELIGHT);
        }
        //Logger.recordOutput("Backpose", backPose.pose);


        if (backPose != null)  {
            rawFiducials = backPose.rawFiducials;
        } else {
            rawFiducials = emptyFiducials;
        }

        double deltaSeconds = Timer.getFPGATimestamp() - lastOdometryTime;
        //Logger.recordOutput("LL Pose Pre-Validation", backPose == null ? nilPose : backPose.pose);
        //Logger.recordOutput("LL MegaTag2", megaTag2Pose == null ? nilPose : megaTag2Pose.pose);
        backPose = validatePoseEstimate(backPose, deltaSeconds);
        PoseEstimate frontPose = validatePoseEstimate(null, 0);//disabled
        
        if (frontPose != null && backPose != null) {
            bestPose = (frontPose.avgTagArea >= backPose.avgTagArea) ? frontPose : backPose;
        } else if (frontPose != null) {
            bestPose = frontPose;
        } else {
            bestPose = backPose;
        }

        lastPose = bestPose;

        
        
        if (bestPose != null) {
            lastOdometryTime = Timer.getFPGATimestamp();
            lastPoseEstimate = bestPose;
            if (odometryEnabled) {
                //RobotContainer.instance.drivetrain.addVisionMeasurement(bestPose.pose, bestPose.timestampSeconds);
                RobotState.getInstance().addVisionUpdate(
                    new VisionUpdate(
                            bestPose.timestampSeconds,
                            new com.team254.lib.geometry.Translation2d(bestPose.pose.getTranslation()),
                            new com.team254.lib.geometry.Translation2d(0,0),//mConstants.kRobotToCamera.getTranslation(), // Use the correct camera offset
                            0.02
                    )
                );
            }
        }//-0.16342

        //Logger.recordOutput("LL Pose Valid?", bestPose != null);
        //Logger.recordOutput("LL Pose", bestPose == null ? nilPose : bestPose.pose);
        //Logger.recordOutput("LL Pose Avg Tag Dist", bestPose == null ? -1 : bestPose.avgTagDist);
        //Logger.recordOutput("LL Pose Avg Tag Area", bestPose == null ? -1 : bestPose.avgTagArea);

        Pose2d robotPose = Drive.getInstance().getPose().toLegacy();

        double targetId = LimelightHelpers.getFiducialID(BACK_LIMELIGHT);
        //Logger.recordOutput("AprilTag ID", targetId);

        //Logger.recordOutput("Limelight/BackPose", backPose.pose);
        
    }

    public void setAutoTagFilter() {
        LimelightHelpers.SetFiducialIDFiltersOverride(BACK_LIMELIGHT, autoTagFilter);
    }

    public void setTeleopTagFilter() {
        LimelightHelpers.SetFiducialIDFiltersOverride(BACK_LIMELIGHT, teleopTagFilter);
    }

    public RawFiducial getFiducial(int id) {
        for (RawFiducial tag : rawFiducials) {
            if (tag != null && tag.id == id) {
                return tag;
            }
        }

        return null;
    }

    public RawFiducial getFiducialRange(int low, int high) {
        for (RawFiducial tag : rawFiducials) {
            if (tag != null && low <= tag.id && tag.id <= high) {
                return tag;
            }
        }

        return null;
    }

    public RawFiducial getBiggestFiducialRange(int low, int high) {
        RawFiducial biggest = null;
        double area = -1;
        for (RawFiducial tag : rawFiducials) {
            if (tag != null && low <= tag.id && tag.id <= high) {
                if (biggest == null || tag.ta > area) {
                    biggest = tag;
                    area = tag.ta;
                }
            }
        }
        return biggest;
    }

    public boolean isLimelightConnected() {
        return limelightConnected;
    }

    public Pose2d getCurrentTrapPose() {
        return currentTrapPose;
    }

    public void setAllowVisionOdometry(boolean odometryEnabled) {
        this.odometryEnabled = odometryEnabled;
    }
    
    public Command allowVisionOdometry(boolean odometryEnabled) {
        return Commands.runOnce(() -> setAllowVisionOdometry(odometryEnabled));
    }

    public void setDirectTagAiming(boolean directTagAiming) {
        this.directTagAiming = directTagAiming;
    }

    public Command directTagAiming(boolean directTagAiming) {
        return Commands.runOnce(() -> setDirectTagAiming(directTagAiming));
    }

    public Double getDistanceOverride() {
        return distanceOverride;
    }

    public void setDistanceOverride(Double distance) {
        distanceOverride = distance;
    }

    public Command clearDistanceOverride() {
        return distanceOverride(null);
    }

    public Command distanceOverride(Double distance) {
        return Commands.runOnce(() -> setDistanceOverride(distance));
    }

    public void setTagHeightOverride(Double height) {
        tagHeightOverride = height;
    }

    public Command clearTagHeightOverride() {
        return tagHeightOverride(null);
    }

    public Command tagHeightOverride(Double height) {
        return Commands.runOnce(() -> setTagHeightOverride(height));
    }

    public void setMegatag2Enabled(boolean enabled) {
        megatag2Enabled = enabled;
    }

    public Command megatag2Enabled(boolean enabled) {
        return Commands.runOnce(() -> setMegatag2Enabled(enabled));
    }
    
    
    //@Override
    //public void outputTelemetry(){
    //    Logger.recordOutput("Limelight/LL Connected", limelightConnected);
    //    Logger.recordOutput("Limelight/BackPose", bestPose.pose);

    //} 

    public PoseEstimate validatePoseEstimate(PoseEstimate poseEstimate, double deltaSeconds) {
        if (poseEstimate == null) return null;

        if (megatag2Enabled) {
            if (poseEstimate.tagCount == 0) return null;
            if (Math.abs(Drive.getInstance().getPigeonRate()) > 720) return null;
        } else {
            double tagMin = 1;
            double tagMax = 2;
            double minArea = 0.08;
            if (poseEstimate.tagCount == 1) minArea = 0.18;
            if (poseEstimate.tagCount > tagMax || poseEstimate.tagCount < tagMin) return null;
            if (poseEstimate.avgTagArea < minArea) return null;
            if (poseEstimate.avgTagDist > 6) return null;

        }

        return poseEstimate;
    }

    public class LimelightPose {
        public final Pose2d pose;
        public final double timestamp;
        public final double targetArea;

        public LimelightPose(Pose2d pose, double timestamp, double targetArea) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.targetArea = targetArea;
        }
    }
}

