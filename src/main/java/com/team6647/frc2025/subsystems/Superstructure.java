package com.team6647.frc2025.subsystems;

import com.team1678.frc2024.Constants1678;
import com.team1678.frc2024.Ports1678;
import com.team1678.frc2024.controlboard.ControlBoard;
import com.team1678.frc2024.led.TimedLEDState;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.CoralPivot;
import com.team1678.frc2024.subsystems.PeriodicLogs;
import com.team1678.frc2024.subsystems.Subsystem;
import com.team1678.lib.TunableNumber;
import com.team1678.lib.drivers.BeamBreak;
import com.team1678.lib.requests.IfRequest;
import com.team1678.lib.requests.LambdaRequest;
import com.team1678.lib.requests.ParallelRequest;
import com.team1678.lib.requests.Request;
import com.team1678.lib.requests.SequentialRequest;
import com.team1678.lib.requests.WaitForPrereqRequest;
import com.team1678.lib.requests.WaitRequest;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import com.team254.lib.util.TimeDelayedBoolean;
import com.team6647.frc2025.FieldLayout;
import com.team6647.frc2025.Robot;
import com.team6647.frc2025.FieldLayout.CoralTarget;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.logging.Level;

import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends Subsystem {

	private static Superstructure mInstance;

	public static synchronized Superstructure getInstance() {
		if (mInstance == null) {
			mInstance = new Superstructure();
		}

		return mInstance;
	}

	// Request tracking variables
	private Request activeRequest = null;
	private ArrayList<Request> queuedRequests = new ArrayList<>(0);
	private boolean hasNewRequest = false;
	private boolean allRequestsComplete = false;

	// Subsystems

	// LEDs
	// private final LEDs mLEDs = LEDs.getInstance();
	private TimedLEDState mHeldState = TimedLEDState.NOTE_HELD_SHOT;

	// Target tracking
	private Drive mDrive = Drive.getInstance();
	private Elevator mElevator = Elevator.getInstance();
	private CoralPivot mCoralPivot = CoralPivot.getInstance();
	private double mDistanceToTarget = 0.0;
	private double mAngularErrToTarget = 0.0;

	// Manual param tuning
	// public final boolean kUseSmartdash = false;
	// public TunableNumber kCurveTuner = new
	// TunableNumber("FiringParams/ManualCurveTune", 0.0, true);
	// public TunableNumber kSkewTuner = new
	// TunableNumber("FiringParams/ManualSkewTune", 0.0, true);
	// public TunableNumber mHoodTuner = new
	// TunableNumber("FiringParams/ManualHoodTune", 0.0, true);
	// public TunableNumber mRPMTuner = new
	// TunableNumber("FiringParams/ManualRPMTune", 0.0, true);

	// Trackers
	// private boolean CLIMB_MODE = false;
	// private boolean PREP = false;
	// private boolean FERRY_SHOT = false;
	// private boolean WANTS_SPINDOWN = false;

	// Corals
	public CoralTarget angles[] = { CoralTarget.RIGHT, CoralTarget.BOTTOM_RIGHT, CoralTarget.BOTTOM_LEFT,
			CoralTarget.LEFT, CoralTarget.TOP_LEFT, CoralTarget.TOP_RIGHT };
	public int coralId = 3;
	public int level = 3;
	public Levels currentLevel = Levels.LEVEL3;
	public int subCoralId = 1;
	public int coralStationPosition = 0;
	private CoralRoller mCoralRoller = CoralRoller.getInstance();

	public boolean placing_coral = true;

	public void setLevel(int level) {
		this.level = level;
		if (level == 1) {
			this.currentLevel = Levels.LEVEL1;
			mCoralRoller.OUTAKING = CoralRoller.State.OUTAKING1;
		}
		if (level == 2) {
			this.currentLevel = Levels.LEVEL2;
			mCoralRoller.OUTAKING = CoralRoller.State.OUTAKING2;
		}
		if (level == 3) {
			this.currentLevel = Levels.LEVEL3;
			mCoralRoller.OUTAKING = CoralRoller.State.OUTAKING3;
		}
		if (level == 4) {
			this.currentLevel = Levels.LEVEL4;
			mCoralRoller.OUTAKING = CoralRoller.State.OUTAKING4;
		}
	}

	public Pose2d sourcePose1 = new Pose2d(1.13,7.06,Rotation2d.fromDegrees(-53.4));
	
	public Pose2d sourcePose2 = new Pose2d(1.2310409545898438,0.873404324054718,Rotation2d.fromDegrees(55));

	public Pose2d sourcePose = sourcePose1;

	public void setSource(int id) {
		if(id == 1){

		}
	}

	public boolean requestsCompleted() {
		return allRequestsComplete;
	}

	public void request(Request r) {
		setActiveRequest(r);
		clearRequestQueue();
	}

	private void setActiveRequest(Request request) {
		activeRequest = request;
		hasNewRequest = true;
		allRequestsComplete = false;
	}

	private void clearRequestQueue() {
		queuedRequests.clear();
	}

	private void setRequestQueue(List<Request> requests) {
		clearRequestQueue();
		for (Request req : requests) {
			queuedRequests.add(req);
		}
	}

	private void setRequestQueue(Request activeRequest, ArrayList<Request> requests) {
		request(activeRequest);
		setRequestQueue(requests);
	}

	public void addRequestToQueue(Request req) {
		queuedRequests.add(req);
		allRequestsComplete = false;
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				clearRequestQueue();
			}

			@Override
			public void onLoop(double timestamp) {
				try {
					if (hasNewRequest && activeRequest != null) {
						activeRequest.act();
						hasNewRequest = false;
					}

					if (activeRequest == null) {
						if (queuedRequests.isEmpty()) {
							allRequestsComplete = true;
						} else {
							request(queuedRequests.remove(0));
						}
					} else if (activeRequest.isFinished()) {
						activeRequest = null;
					}

					// updateShootingSetpoints();
				} catch (Exception e) {
					e.printStackTrace();
				}
				PeriodicLogs.periodic();
			}

			@Override
			public void onStop(double timestamp) {
				stop();
			}
		});
	}

	@Override
	public void stop() {
		activeRequest = null;
		clearRequestQueue();
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public void readPeriodicInputs() {

	}

	@Override
	public void outputTelemetry() {

		// SmartDashboard.putBoolean("Ferry Shot", FERRY_SHOT);

		SmartDashboard.putNumber("FiringParams/Angular Err To Target", mAngularErrToTarget);
	}

	/* Superstructure functions */

	public synchronized void showAngle() {
		Logger.recordOutput("CoralPose", FieldLayout.getCoralTargetPos(angles[coralId]).corals[subCoralId].toLegacy());
		Logger.recordOutput("RealCoralPose", FieldLayout.getCoralTargetPos(angles[coralId]).realCorals[subCoralId].toLegacy());
		Logger.recordOutput("SourcePose", sourcePose.toLegacy());
		Logger.recordOutput("Coral/CoralId", coralId);
		Logger.recordOutput("Coral/SubCoralId", subCoralId);
	}

	public synchronized void showSource() {
		Logger.recordOutput("CoralSource", FieldLayout.getCoralStation(true, coralStationPosition).toLegacy());
	}

	private double levelCenter = 8.77400016784668;
	private Pose2d[] levels = {
			Pose2d.fromTranslation(new Translation2d(levelCenter, 1.3818594217300415)),
			Pose2d.fromTranslation(new Translation2d(levelCenter, 3.002558469772339)),
			Pose2d.fromTranslation(new Translation2d(levelCenter, 5.058445453643799)),
			Pose2d.fromTranslation(new Translation2d(levelCenter, 6.664137363433838)),
	};

	public synchronized void showLevel() {
		if (level == 1) {
			Logger.recordOutput("Pointers/PointerC", levels[0].toLegacy());
			Logger.recordOutput("Pointers/Pointer1", levels[1].toLegacy());
			Logger.recordOutput("Pointers/Pointer2", levels[2].toLegacy());
			Logger.recordOutput("Pointers/Pointer3", levels[3].toLegacy());
		}
		if (level == 2) {
			Logger.recordOutput("Pointers/Pointer1", levels[0].toLegacy());
			Logger.recordOutput("Pointers/PointerC", levels[1].toLegacy());
			Logger.recordOutput("Pointers/Pointer2", levels[2].toLegacy());
			Logger.recordOutput("Pointers/Pointer3", levels[3].toLegacy());
		}
		if (level == 3) {
			Logger.recordOutput("Pointers/Pointer1", levels[0].toLegacy());
			Logger.recordOutput("Pointers/Pointer2", levels[1].toLegacy());
			Logger.recordOutput("Pointers/PointerC", levels[2].toLegacy());
			Logger.recordOutput("Pointers/Pointer3", levels[3].toLegacy());
		}
		if (level == 4) {
			Logger.recordOutput("Pointers/Pointer1", levels[0].toLegacy());
			Logger.recordOutput("Pointers/Pointer2", levels[1].toLegacy());
			Logger.recordOutput("Pointers/Pointer3", levels[2].toLegacy());
			Logger.recordOutput("Pointers/PointerC", levels[3].toLegacy());
		}
	}

	// spotless:off

	/**
	 * BeamBreak Sensor reading.
	 * 
	 * @param mBreak       BeamBreak Sensor.
	 * @param target_state If wanted reading is true (broken) or false (not broken).
	 * 
	 * @return Boolean for if target state is acheived.
	 */
	private Request breakWait(BeamBreak mBreak, boolean target_state) {
		return new Request() {

			@Override
			public void act() {
			}

			@Override
			public boolean isFinished() {
				return mBreak.get() == target_state;
			}
		};
	}

	private Request boolWait(BooleanSupplier func) {
		return new Request() {

			@Override
			public void act() {
			}

			@Override
			public boolean isFinished() {
				return func.getAsBoolean();
			}
		};
	}

	/**
	 * Debounced BeamBreak Sensor reading.
	 * 
	 * @param mBreak               BeamBreak Sensor.
	 * @param target_state         If wanted reading is true (broken) or false (not
	 *                             broken).
	 * @param delayed_wait_seconds Debounces time from a BeamBreak Sensor.
	 * 
	 * @return Boolean for if target state is acheived after debouncing the signal.
	 */
	private Request breakWait(BeamBreak mBreak, boolean target_state, double delayed_wait_seconds) {
		return new Request() {

			TimeDelayedBoolean timeout = new TimeDelayedBoolean();

			@Override
			public void act() {
			}

			@Override
			public boolean isFinished() {
				return timeout.update(mBreak.get() == target_state, delayed_wait_seconds);
			}
		};
	}

	/**
	 * Stop Intake Rollers, Serializer, Feeder, and Amp Rollers.
	 */
	private Request idleRequest() {
		return new ParallelRequest(

		);
	}

	public enum Levels {
		LEVEL1(Elevator.kL1Height, CoralPivot.kLevel1Angle),
		LEVEL2(Elevator.kL2Height, CoralPivot.kLevel2Angle),
		LEVEL3(Elevator.kL3Height, CoralPivot.kLevel3Angle),
		LEVEL4(Elevator.kL4Height, CoralPivot.kLevel4Angle),

		ALGAEING1(Elevator.kAlgaeing1, CoralPivot.kAlgaeing1),
		ALGAEING2(Elevator.kAlgaeing2, CoralPivot.kAlgaeing2);


		public double elevatorHeight;
		public double coralAngle;
		public double rollerSpeed;

		private Levels(double elevatorHeight, double coralAngle) {
			this.elevatorHeight = elevatorHeight;
			this.coralAngle = coralAngle;
			this.rollerSpeed = rollerSpeed;
		}
	}

	public Request prepareLevel(Levels levelPos) {
		return new ParallelRequest(
				mElevator.setPositionRequest(levelPos.elevatorHeight),
				mCoralPivot.setPositionRequest(levelPos.coralAngle));
	}

	public Request softHome() {
		return new ParallelRequest(
				// mElevator.LRequest(Levels.LEVEL1),
				new LambdaRequest(
						() -> {
							// clearRequestQueue();
							mElevator.setOpenLoop(0);
							mElevator.setWantHome(true);
							// mElevator.LRequest(Levels.LEVEL1);
						}),
				mCoralPivot.LRequest(Levels.LEVEL1),
			new WaitForPrereqRequest(()->!mElevator.isHoming())
				);
	}

	/**
	 * Stop Intake Rollers, Serializer, Feeder, and Amp Rollers.
	 */
	public void idleState() {
		request(idleRequest());
	}

	ControlBoard mControlBoard = ControlBoard.getInstance();

	public void preGen() {

	}

	public Pose2d getActiveCoral(){
		return FieldLayout.getCoralTargetPos(Superstructure.getInstance().angles[coralId]).realCorals[subCoralId];
	}

	public Pose2d getActiveCoralL4(){
		return FieldLayout.getCoralTargetPos(Superstructure.getInstance().angles[coralId]).corals4[subCoralId];
	}

	// spotless:on
}
