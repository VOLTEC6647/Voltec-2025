
package com.team6647.frc2025.subsystems.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.team1678.frc2024.RobotState;
import com.team1678.frc2024.RobotState.VisionUpdate;
import com.team1678.frc2024.subsystems.Drive;
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

public class VisionSubsystem extends SubsystemBase{

    public static final String FRONT_LIMELIGHT = "limelight-front";
    public static final String BACK_LIMELIGHT = "limelight";
    private static final double FALLBACK_SPEAKER_DISTANCE = 1.357;
    private static final double FALLBACK_TAG_HEIGHT = 461.37;
    private static final double NOTE_VELOCITY = 10;
    public static final double SHOOTER_DEGREE_OFFSET = -6; // To account for shooter curving to the left

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
    private boolean limelightConnected = false;
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
    private Double stageAngle = null;
    private Pose2d currentTrapPose = null;
    public Double stageTX = null;
    public Double stageTY = null;
    private boolean megatag2Enabled = false;
    public HashMap<Integer, Pose2d> trapPoses = new HashMap<>();

    private final RawFiducial[] emptyFiducials = new RawFiducial[0];
    public RawFiducial[] rawFiducials = emptyFiducials;

    private boolean firstPeriodic = true;
    private final Transform2d speakerOffset = new Transform2d(.1524, 0, new Rotation2d());
    private final Transform2d trapPoseTransform = new Transform2d(1.132, 0.32, Rotation2d.fromRadians(-0.005));
    private final Pose2d nilPose = new Pose2d(-1, -1, new Rotation2d());

    final int[] autoTagFilter = new int[] { 3, 4, 7, 8 };
    final int[] teleopTagFilter = new int[] { 3, 4, 5, 6, 7, 8, 11, 12, 13, 14, 15, 16 };
    
    public VisionSubsystem() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (Exception e) {
            aprilTagFieldLayout = null;
        }

        SmartDashboard.putNumber("BLUE Distance Offset", 0);
        SmartDashboard.putNumber("RED Distance Offset", 0);

        SmartDashboard.putNumber("RED Pixel Offset", 0);
        SmartDashboard.putNumber("BLUE Pixel Offset", 0);       
    }

    @Override
    public void periodic() {
        if (firstPeriodic) {
            for (int i = 11; i <= 16; i++) {
                Logger.recordOutput("Trap Poses/" + i, trapPoses.get(i));
            }
            //RobotContainer.instance.drivetrain.addVisionMeasurement(new Pose2d(), Timer.getFPGATimestamp());
            firstPeriodic = false;
        }

        Logger.recordOutput("Odometry Enabled", odometryEnabled);
        
        double newHeartbeat = LimelightHelpers.getLimelightNTDouble("limelight", "hb");
        if (newHeartbeat > limelightHeartbeat) {
            limelightConnected = true;
            limelightHeartbeat = newHeartbeat;
            lastHeartbeatTime = Timer.getFPGATimestamp();
        } else if (Timer.getFPGATimestamp() - lastHeartbeatTime >= 1) {
            limelightConnected = false;
        }
        if (Robot.isSimulation()) limelightConnected = true;
        Logger.recordOutput("LL Connected", limelightConnected);

        if (!limelightConnected) {
            rawFiducials = emptyFiducials;
            speakerAngle = null;
            speakerDistance = null;
            speakerTagHeight = null;
            Logger.recordOutput("LL Pose Valid?", false);
            Logger.recordOutput("LL Pose", nilPose);
            Logger.recordOutput("Speaker Angle", -1);
            Logger.recordOutput("Speaker Distance", -1);
            Logger.recordOutput("Speaker Tag Height", -1);
            Logger.recordOutput("Can See Speaker", false);
            return;
        }


        PoseEstimate bestPose;
        PoseEstimate backPose;
        PoseEstimate megaTag2Pose = null;

        
        if (megatag2Enabled) {
            double megatagDegrees = Drive.getInstance().getHeading().getDegrees();
            if (Robot.is_red_alliance) megatagDegrees = MathUtil.inputModulus(megatagDegrees + 180, -180, 180);
            LimelightHelpers.SetRobotOrientation(BACK_LIMELIGHT, megatagDegrees, 0, 0, 0, 0, 0);
            backPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(BACK_LIMELIGHT);
        } else {
            backPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(BACK_LIMELIGHT);
        }

        if (backPose != null)  {
            rawFiducials = backPose.rawFiducials;
        } else {
            rawFiducials = emptyFiducials;
        }

        double deltaSeconds = Timer.getFPGATimestamp() - lastOdometryTime;
        Logger.recordOutput("LL Pose Pre-Validation", backPose == null ? nilPose : backPose.pose);
        Logger.recordOutput("LL MegaTag2", megaTag2Pose == null ? nilPose : megaTag2Pose.pose);
        backPose = validatePoseEstimate(backPose, deltaSeconds);
        PoseEstimate frontPose = validatePoseEstimate(null, 0);//disabled
        
        if (frontPose != null && backPose != null) {
            bestPose = (frontPose.avgTagArea >= backPose.avgTagArea) ? frontPose : backPose;
        } else if (frontPose != null) {
            bestPose = frontPose;
        } else {
            bestPose = backPose;
        }

        
        
        if (bestPose != null) {
            lastOdometryTime = Timer.getFPGATimestamp();
            lastPoseEstimate = bestPose;
            if (odometryEnabled) {
                //RobotContainer.instance.drivetrain.addVisionMeasurement(bestPose.pose, bestPose.timestampSeconds);
                RobotState.getInstance().addVisionUpdate(
                    new VisionUpdate(
                            bestPose.timestampSeconds,
                            new com.team254.lib.geometry.Translation2d(bestPose.pose.getTranslation()),
                            new com.team254.lib.geometry.Translation2d(),//mConstants.kRobotToCamera.getTranslation(), // Use the correct camera offset
                            0.02
                    )
                );
            }
        }

        Logger.recordOutput("LL Pose Valid?", bestPose != null);
        Logger.recordOutput("LL Pose", bestPose == null ? nilPose : bestPose.pose);
        Logger.recordOutput("LL Pose Avg Tag Dist", bestPose == null ? -1 : bestPose.avgTagDist);
        Logger.recordOutput("LL Pose Avg Tag Area", bestPose == null ? -1 : bestPose.avgTagArea);

        Pose2d robotPose = Drive.getInstance().getPose().toLegacy();

        double targetId = LimelightHelpers.getFiducialID(BACK_LIMELIGHT);
        Logger.recordOutput("AprilTag ID", targetId);

        
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

    public Pose2d calculateTrapPose(int tagID) {
        Optional<Pose3d> pose3d = aprilTagFieldLayout.getTagPose(tagID);
        if (pose3d.isEmpty()) return null;

        return pose3d.get().toPose2d().transformBy(trapPoseTransform);
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

