// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2025.Constants6328;
import org.littletonrobotics.frc2025.Constants6328.Mode;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.frc2025.util.swerve.SwerveSetpoint;
import org.littletonrobotics.frc2025.util.swerve.SwerveSetpointGenerator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team1678.frc2024.Constants1678;
import com.team1678.lib.Util;
import com.team6647.frc2025.Constants.ElevatorConstants;
import com.team6647.frc2025.Robot;
import com.team6647.frc2025.subsystems.Elevator;

import choreo.trajectory.SwerveSample;

public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private static final LoggedTunableNumber coastWaitTime =
      new LoggedTunableNumber("Drive/CoastWaitTimeSeconds", 0.5);
  private static final LoggedTunableNumber coastMetersPerSecondThreshold =
      new LoggedTunableNumber("Drive/CoastMetersPerSecThreshold", .05);

  private final Timer lastMovementTimer = new Timer();

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.moduleTranslations);

  @AutoLogOutput private boolean velocityMode = false;
  @AutoLogOutput private boolean brakeModeEnabled = true;

  private SwerveSetpoint currentSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          });
  private final SwerveSetpointGenerator swerveSetpointGenerator;

  private static Drive mInstance;

  public static Drive getInstance(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
		if (mInstance == null) {
			mInstance = new Drive(gyroIO,flModuleIO,frModuleIO,blModuleIO,brModuleIO);
		}
		return mInstance;
	}

  public static Drive getInstance(){
    return mInstance;
  }

  private double kPathFollowDriveP;
  private double kPathFollowTurnP;
  private PIDController choreoX;
  private PIDController choreoY;
  private PIDController choreoRotation;
  public int acceptingHeading;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
    lastMovementTimer.start();
    setBrakeMode(true);

    swerveSetpointGenerator =
        new SwerveSetpointGenerator(kinematics, DriveConstants.moduleTranslations);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Choreo init
    kPathFollowDriveP = 2.5;
    kPathFollowTurnP = 1.8;//1.8
    choreoX = new PIDController(kPathFollowDriveP, 0, 0);
    choreoY = new PIDController(kPathFollowDriveP, 0, 0);
    choreoRotation = new PIDController(kPathFollowTurnP, 0, 0);
    choreoRotation.enableContinuousInput(-Math.PI, Math.PI);

    try{
      RobotConfig config = RobotConfig.fromGUISettings();
      RobotState state = RobotState.getInstance();
	  AutoBuilder.configure(
			state::getEstimatedPose, // Robot pose supplier
			state::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
			this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
			(speeds, feedforwards) -> runVelocity(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
			new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
					new PIDConstants(1, 0.0, 0.0), // Translation PID constants
					new PIDConstants(2, 0.0, 0.0) // Rotation PID constants
			),
			config, // The robot configuration
			() -> {
			  var alliance = DriverStation.getAlliance();
			  if (alliance.isPresent()) {
				return alliance.get() == DriverStation.Alliance.Red;
			  }
			  return false;
			},
			this // Reference to this subsystem to set requirements
	);
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  public enum CoastRequest {
    AUTOMATIC,
    ALWAYS_BREAK,
    ALWAYS_COAST
  }

  @Setter @AutoLogOutput private CoastRequest coastRequest = CoastRequest.AUTOMATIC;

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    LoggedTracer.record("Drive/Inputs");

    // Call periodic on modules
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", new SwerveModuleState[] {});
    }

    // Send odometry updates to robot state
    double[] sampleTimestamps =
        Constants6328.getMode() == Mode.SIM
            ? new double[] {Timer.getTimestamp()}
            : gyroInputs.odometryYawTimestamps; // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
      for (int j = 0; j < 4; j++) {
        wheelPositions[j] = modules[j].getOdometryPositions()[i];
      }
      RobotState.getInstance()
          .addOdometryObservation(
              new RobotState.OdometryObservation(
                  wheelPositions,
                  Optional.ofNullable(
                      gyroInputs.data.connected() ? gyroInputs.odometryYawPositions[i] : null),
                  sampleTimestamps[i]));

      // Log 3D robot pose
      Logger.recordOutput(
          "RobotState/EstimatedPose3d",
          new Pose3d(RobotState.getInstance().getEstimatedPose())
              .exp(
                  new Twist3d(
                      0.0,
                      0.0,
                      Math.abs(gyroInputs.data.pitchPosition().getRadians())
                          * DriveConstants.trackWidthX
                          / 2.0,
                      0.0,
                      gyroInputs.data.pitchPosition().getRadians(),
                      0.0))
              .exp(
                  new Twist3d(
                      0.0,
                      0.0,
                      Math.abs(gyroInputs.data.rollPosition().getRadians())
                          * DriveConstants.trackWidthY
                          / 2.0,
                      gyroInputs.data.rollPosition().getRadians(),
                      0.0,
                      0.0)));
    }

    RobotState.getInstance().addDriveSpeeds(getChassisSpeeds());
    RobotState.getInstance().setPitch(gyroInputs.data.pitchPosition());
    RobotState.getInstance().setRoll(gyroInputs.data.rollPosition());

    // Update brake mode
    // Reset movement timer if velocity above threshold
    if (Arrays.stream(modules)
        .anyMatch(
            (module) ->
                Math.abs(module.getVelocityMetersPerSec()) > coastMetersPerSecondThreshold.get())) {
      lastMovementTimer.reset();
    }

    if (DriverStation.isEnabled()) {
      coastRequest = CoastRequest.AUTOMATIC;
    }

    switch (coastRequest) {
      case AUTOMATIC -> {
        if (DriverStation.isEnabled()) {
          setBrakeMode(true);
        } else if (lastMovementTimer.hasElapsed(coastWaitTime.get())) {
          setBrakeMode(false);
        }
      }
      case ALWAYS_BREAK -> {
        setBrakeMode(true);
      }
      case ALWAYS_COAST -> {
        setBrakeMode(false);
      }
    }

    // Update current setpoint if not in velocity mode
    if (!velocityMode) {
      currentSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates());
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(
        !gyroInputs.data.connected() && Constants6328.getMode() != Mode.SIM && !Robot.isJITing());

    // Record cycle time
    LoggedTracer.record("Drive/Periodic");
  }

  /** Set brake mode to {@code enabled} doesn't change brake mode if already set. */
  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled != enabled) {
      Arrays.stream(modules).forEach(module -> module.setBrakeMode(enabled));
    }
    brakeModeEnabled = enabled;
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    velocityMode = true;
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants6328.loopPeriodSecs);
    SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);
    currentSetpoint =
        swerveSetpointGenerator.generateSetpoint(
            DriveConstants.moduleLimitsFree,
            currentSetpoint,
            discreteSpeeds,
            Constants6328.loopPeriodSecs);
    SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized);
    Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }
  }

  /**
   * Runs the drive at the desired velocity with setpoint module forces.
   *
   * @param speeds Speeds in meters/sec
   * @param moduleForces The forces applied to each module
   */
  public void runVelocity(ChassisSpeeds speeds, List<Vector<N2>> moduleForces) {
    velocityMode = true;
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants6328.loopPeriodSecs);
    SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);
    currentSetpoint =
        swerveSetpointGenerator.generateSetpoint(
            DriveConstants.moduleLimitsFree,
            currentSetpoint,
            discreteSpeeds,
            Constants6328.loopPeriodSecs);
    SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized);
    Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());

    // Save module forces to swerve states for logging
    SwerveModuleState[] wheelForces = new SwerveModuleState[4];
    // Send setpoints to modules
    SwerveModuleState[] moduleStates = getModuleStates();
    for (int i = 0; i < 4; i++) {
      // Optimize state
      Rotation2d wheelAngle = moduleStates[i].angle;
      setpointStates[i].optimize(wheelAngle);
      setpointStates[i].cosineScale(wheelAngle);

      // Calculate wheel torque in direction
      var wheelForce = moduleForces.get(i);
      Vector<N2> wheelDirection = VecBuilder.fill(wheelAngle.getCos(), wheelAngle.getSin());
      double wheelTorqueNm = wheelForce.dot(wheelDirection) * DriveConstants.wheelRadius;
      modules[i].runSetpoint(setpointStates[i], wheelTorqueNm);

      // Save to array for logging
      wheelForces[i] = new SwerveModuleState(wheelTorqueNm, setpointStates[i].angle);
    }
    Logger.recordOutput("Drive/SwerveStates/ModuleForces", wheelForces);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    velocityMode = false;
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns the module states (turn angles and drive velocities) for all the modules. */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "Drive/SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the raw gyro rotation read by the IMU */
  public Rotation2d getGyroRotation() {
    return gyroInputs.data.yawPosition();
  }

  public static class KinematicLimits {
		public double kMaxDriveVelocity = Constants1678.SwerveConstants.maxSpeed; // m/s
		public double kMaxAccel = Double.MAX_VALUE; // m/s^2
		public double kMaxAngularVelocity = Constants1678.SwerveConstants.maxAngularVelocity; // rad/s
		public double kMaxAngularAccel = Double.MAX_VALUE; // rad/s^2
	}

  public void resetHeadings(){
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = Rotation2d.fromDegrees(0);
    }
    kinematics.resetHeadings(headings);
    stop();
  }
  
  public void choreoController(SwerveSample sample) {
	//lastSample = sample.getPose();

	Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
	ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
	  sample.vx + choreoX.calculate(currentPose.getTranslation().getX(), sample.x),
	  sample.vy + choreoY.calculate(currentPose.getTranslation().getY(), sample.y),
	  sample.omega + choreoRotation.calculate(currentPose.getRotation().getRadians(), sample.heading), //acceptingHeading
	  currentPose.getRotation()
	);
	runVelocity(speeds);
  }

}
