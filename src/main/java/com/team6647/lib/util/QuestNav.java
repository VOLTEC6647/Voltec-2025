package com.team6647.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.networktables.TimestampedFloatArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

/** IO implementation for real Limelight hardware. */
public class QuestNav {
  public record QuestNavData(
      Pose2d pose,
      double batteryPercent,
      double timestamp,
      float[] translation,
      float[] rotation) {
  }

  private static QuestNav mInstance;
    
    public static QuestNav getInstance() {

		return mInstance;
	}

  // Configure Network Tables topics (questnav/...) to communicate with the Quest
  // HMD
  private NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  private NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi = nt4Table
      .getIntegerTopic("mosi")
      .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position")
      .subscribe(new float[] { 0.0f, 0.0f, 0.0f });
  private FloatArraySubscriber questAngles = nt4Table.getFloatArrayTopic("eulerAngles")
      .subscribe(new float[] { 0.0f, 0.0f, 0.0f });
  private DoubleSubscriber questBatteryPercent = nt4Table.getDoubleTopic("device/batteryPercent").subscribe(0.0f);

  private BooleanSubscriber questIsTracking = nt4Table.getBooleanTopic("device/isTracking").subscribe(false);
  private IntegerSubscriber questTrackingLostCount = nt4Table.getIntegerTopic("device/trackingLostCounter")
      .subscribe(0);

  /** Subscriber for heartbeat requests */
  private final DoubleSubscriber heartbeatRequestSub = nt4Table.getDoubleTopic("heartbeat/quest_to_robot")
      .subscribe(0.0);
  /** Publisher for heartbeat responses */
  private final DoublePublisher heartbeatResponsePub = nt4Table.getDoubleTopic("heartbeat/robot_to_quest").publish();
  /** Last processed heartbeat request ID */
  private double lastProcessedHeartbeatId = 0;

  private final Transform2d robotToCamera;

  private Translation3d[] questNavRawToFieldCoordinateSystemQueue = new Translation3d[15];
  private Translation2d questNavRawToFieldCoordinateSystem = new Translation2d();

  int count = 0;
  int idx = 0;

  protected Rotation2d gyroResetAngle = new Rotation2d();
  protected Pose2d lastPose3d = new Pose2d();

  public QuestNav(Transform2d robotToCamera) {
    // Initialize the camera to robot transform
    this.robotToCamera = robotToCamera;
    mInstance = this;
  }

  public QuestNavData getQuestNavData() {
    // TimestampedDouble[] timestamps = questTimestamp.get();
    float[] positions = questPosition.get();
    float[] angles = questAngles.get();
    // TimestampedDouble[] battery = questBatteryPercent.readQueue();

    double battery = getBatteryPercent();

    QuestNavData data = new QuestNavData(
        getQuestNavPose(positions, angles).plus(robotToCamera.inverse()),
        battery,
        questTimestamp.get(),
        positions,
        angles);

    if (data.pose.getTranslation().getX() == 0.0 && data.pose.getTranslation().getY() == 0.0) {
      data = null;
    }

    return data;
  }

  // Gets the battery percent of the Quest.
  public double getBatteryPercent() {
    return questBatteryPercent.get();
  }

  // Returns if the Quest is connected.
  public boolean connected() {
    return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
  }

  // Clean up questnav subroutine messages after processing on the headset
  public void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  private Translation2d getQuestNavTranslation(float[] position) {
    return new Translation2d(position[2], -position[0]);
  }

  // Gets the Rotation of the Quest.
  public Rotation2d getQuestNavRotation(float[] angles) {
    return new Rotation2d(
        Units.degreesToRadians(-angles[1]));
  }

  private Pose2d getQuestNavPose(float[] position, float[] angles) {
    var oculousPositionCompensated = getQuestNavTranslation(position); // 6.5
    return new Pose2d(oculousPositionCompensated, getQuestNavRotation(angles));
  }

  /** Process heartbeat requests from Quest and respond with the same ID */
  public void processHeartbeat() {
    double requestId = heartbeatRequestSub.get();
    // Only respond to new requests to avoid flooding
    if (requestId > 0 && requestId != lastProcessedHeartbeatId) {
      heartbeatResponsePub.set(requestId);
      lastProcessedHeartbeatId = requestId;
    }
  }

  public void resetPose(Pose2d pose) {
    questMosi.accept(3);

    questNavRawToFieldCoordinateSystem = pose.getTranslation()
        .minus(lastPose3d.getTranslation().minus(questNavRawToFieldCoordinateSystem));

    count = 0;
    idx = 0;
  }

  public void resetHeading(Rotation2d heading) {
    questMosi.accept(3);

    gyroResetAngle = (lastPose3d.getRotation().minus(gyroResetAngle).minus(heading))
        .unaryMinus();
  }

  public void resetHeading() {
    resetHeading(
        DriverStation.getAlliance().get() == Alliance.Red ? Rotation2d.kPi : Rotation2d.kZero);
  }

  public void resetBlue() {
    resetHeading(Rotation2d.kZero);
  }

  public Boolean getTrackingStatus() {
    return questIsTracking.get();
  }

  // Gets the number of tracking lost events since the Quest connected to the
  // robot.
  public Long getTrackingLostCounter() {
    return questTrackingLostCount.get();
  }
}
