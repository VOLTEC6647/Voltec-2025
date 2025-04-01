
package com.team6647.frc2025.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import org.littletonrobotics.frc2025.RobotState;
import com.team6647.frc2025.Robot;
import com.team6647.frc2025.subsystems.vision.LimelightHelpers.PoseEstimate;
import com.team6647.frc2025.subsystems.vision.LimelightHelpers.RawFiducial;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightPoseEstimator{

    // Adjustable transform for the Limelight pose per-alliance
    public AprilTagFieldLayout aprilTagFieldLayout;
    private boolean odometryEnabled = true;
    private double lastOdometryTime = 0;

    private double limelightHeartbeat = 0;
    private double frontLimelightHeartbeat = 0;
    private double lastHeartbeatTime = 0;
    private double frontLastHeartbeatTime = 0;
    public boolean limelightConnected = false;
    private boolean frontLimelightConnected = false;

    public Double distanceOverride = null;
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

    double newHeartbeat;
    private final String cameraName;
    
    public LimelightPoseEstimator(String name) {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
        } catch (Exception e) {
            aprilTagFieldLayout = null;
        }
        this.cameraName = name;

        SmartDashboard.putNumber("BLUE Distance Offset", 0);
        SmartDashboard.putNumber("RED Distance Offset", 0);

        SmartDashboard.putNumber("RED Pixel Offset", 0);
        SmartDashboard.putNumber("BLUE Pixel Offset", 0);       
    }

    public PoseEstimate getEstimatedPose() {

        //Logger.recordOutput("Odometry Enabled", odometryEnabled);
        
        newHeartbeat = LimelightHelpers.getLimelightNTDouble(cameraName, "hb");
        if (newHeartbeat > limelightHeartbeat) {
            limelightConnected = true;
            limelightHeartbeat = newHeartbeat;
            lastHeartbeatTime = Timer.getFPGATimestamp();
        } else if (Timer.getFPGATimestamp() - lastHeartbeatTime >= 1) {
            limelightConnected = false;
        }
        //Logger.recordOutput("Limelight/LL Connected", limelightConnected);
        if (Robot.isSimulation()) limelightConnected = true;

        if (!limelightConnected) {
            rawFiducials = emptyFiducials;
            //Logger.recordOutput("LL Pose Valid?", false);
            //Logger.recordOutput("LL Pose", nilPose);
            return null;
        }

        PoseEstimate estimatedPose;

        PoseEstimate megaTag2Pose = null;

        
        if (megatag2Enabled) {
            double megatagDegrees = RobotState.getInstance().getHeading().getDegrees();
            if (Robot.is_red_alliance) megatagDegrees = MathUtil.inputModulus(megatagDegrees + 180, -180, 180);
            LimelightHelpers.SetRobotOrientation(cameraName, megatagDegrees, 0, 0, 0, 0, 0);
            estimatedPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
        } else {
            estimatedPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
        }


        if (estimatedPose != null)  {
            rawFiducials = estimatedPose.rawFiducials;
        }else{
            rawFiducials = emptyFiducials;
        }

        double deltaSeconds = Timer.getFPGATimestamp() - lastOdometryTime;
        //Logger.recordOutput("LL Pose Pre-Validation", backPose == null ? nilPose : backPose.pose);
        //Logger.recordOutput("LL MegaTag2", megaTag2Pose == null ? nilPose : megaTag2Pose.pose);
        estimatedPose = validatePoseEstimate(estimatedPose, deltaSeconds);
        
        return estimatedPose;
        
        //-0.16342

        //Logger.recordOutput("LL Pose Valid?", bestPose != null);
        //Logger.recordOutput("LL Pose", bestPose == null ? nilPose : bestPose.pose);
        //Logger.recordOutput("LL Pose Avg Tag Dist", bestPose == null ? -1 : bestPose.avgTagDist);
        //Logger.recordOutput("LL Pose Avg Tag Area", bestPose == null ? -1 : bestPose.avgTagArea);

        //Pose2d robotPose = Drive.getInstance().getPose().toLegacy();

        //double targetId = LimelightHelpers.getFiducialID(BACK_LIMELIGHT);
        //Logger.recordOutput("AprilTag ID", targetId);

        //Logger.recordOutput("Limelight/BackPose", backPose.pose);
        
    }

    public void setAutoTagFilter() {
        LimelightHelpers.SetFiducialIDFiltersOverride(cameraName, autoTagFilter);
    }

    public void setTeleopTagFilter() {
        LimelightHelpers.SetFiducialIDFiltersOverride(cameraName, teleopTagFilter);
    }

    public double getFPS() {
        double pipelineLatency = LimelightHelpers.getLatency_Pipeline(this.cameraName); // tl in ms
        double captureLatency = LimelightHelpers.getLatency_Capture(this.cameraName); // cl in ms

        double totalLatencyMs = pipelineLatency + captureLatency;

        if (totalLatencyMs <= 0) {
            return 0.0;
        }

        double fps = 1000.0 / totalLatencyMs;

        return fps;
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
            //if (Math.abs(Drive.getInstance().getPigeonRate()) > 720) return null;
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