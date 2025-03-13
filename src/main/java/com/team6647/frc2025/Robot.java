// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6647.frc2025;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.Constants1678;
import com.team1678.frc2024.RobotState;
import com.team1678.frc2024.SubsystemManager;
import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeExecutor;
import com.team6647.frc2025.Constants.CoralPivotConstants;
import com.team6647.frc2025.auto.AutoModeSelector;
import com.team6647.frc2025.auto.modes.configuredQuals.L1Attempt;
import com.team6647.frc2025.auto.modes.configuredQuals.LAlgae2;
import com.team6647.frc2025.auto.modes.configuredQuals.Left1;
import com.team6647.frc2025.auto.modes.configuredQuals.Left2;
import com.team6647.frc2025.auto.modes.configuredQuals.simpleForward2;
import com.team6647.frc2025.auto.modes.configuredQuals.simpleForwardC;
import com.team6647.frc2025.auto.modes.configuredQuals.justForwardC;
import com.team6647.frc2025.auto.paths.TrajectoryGenerator;
import com.team1678.frc2024.controlboard.ControlBoard;
import com.team6647.frc2025.controlboard.DriverControls;
import com.team1678.frc2024.loops.CrashTracker;
import com.team1678.frc2024.loops.Looper;
import com.team1678.frc2024.paths.TrajectoryGenerator1678;
import com.team1678.frc2024.subsystems.AlgaeT;
import com.team1678.frc2024.subsystems.Cancoders;
import com.team1678.frc2024.subsystems.Climber;
import com.team1678.frc2024.subsystems.CoralPivot;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.vision.VisionDeviceManager;
import com.team1678.lib.Util;
import com.team1678.lib.logger.LogUtil;
import com.team1678.lib.requests.LambdaRequest;
import com.team1678.lib.requests.ParallelRequest;
import com.team1678.lib.requests.SequentialRequest;
import com.team1678.lib.requests.WaitRequest;
import com.team1678.lib.swerve.ChassisSpeeds;
import com.team1678.lib.wpi.TimedRobot;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory254;
import com.team254.lib.trajectory.timing.TimedState;
import com.team6647.frc2025.subsystems.Elevator;
import com.team6647.frc2025.subsystems.MotorTest;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.Superstructure.Levels;
import com.team6647.frc2025.subsystems.algae_roller.AlgaeRoller;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;
import com.team6647.frc2025.subsystems.Leds.LEDSubsystem;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.util.List;
import java.util.Optional;

public class Robot extends LoggedRobot {

	

	// util instances
	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
	private final ControlBoard mControlBoard = ControlBoard.getInstance();
	private final DriverControls mDriverControls = new DriverControls();

	private static final LEDSubsystem leds = LEDSubsystem.getInstance();

	// the boss
	private final Superstructure mSuperstructure = Superstructure.getInstance();

	// subsystem instances
	private Drive mDrive;
	private Cancoders mCancoders;

	private MotorTest mMotorTest;
	private AlgaeRoller mAlgaeRoller;
	private AlgaeT mAlgaeT;
	private CoralPivot mCoralPivot;
	private CoralRoller mCoralRoller;

	private Elevator mElevator;
	private com.team1678.frc2024.subsystems.Climber mClimber;





	// vision
	private final VisionDeviceManager mVisionDevices = VisionDeviceManager.getInstance();

	// limelight
	//private final Limelight mLimelight = Limelight.getInstance();

	// enabled and disabled loopers
	private final Looper mEnabledLooper = new Looper();
	private final Looper mDisabledLooper = new Looper();

	// auto instances
	private AutoModeExecutor mAutoModeExecutor = new AutoModeExecutor();
	//public static final AutoModeSelector mAutoModeSelector = new AutoModeSelector();
	public static final AutoModeSelector mThreeNoteNoteSelector = new AutoModeSelector();
	public static boolean is_red_alliance = false;
	public static String serial;

	double disable_enter_time = 0.0;
	
	
	
	static {
		Logger.recordMetadata("Voltec", "Betabot");
		if (Robot.isReal()) {
			serial = System.getenv("serialnum");
			Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
			Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
			new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
		} else {
			serial = "";
			//setUseTiming(false); // Run as fast as possible
			//String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
			//Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
			//Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
		}
		Logger.start();
		Constants1678.isComp = serial.startsWith(Constants1678.kCompSerial);
		Constants1678.isEpsilon = serial.startsWith(Constants1678.kEpsilonSerial);
	}

	public Robot() {
		CrashTracker.logRobotConstruction();

		mDrive = Drive.getInstance();

		//mMotorTest = MotorTest.getInstance();
		mAlgaeRoller = AlgaeRoller.getInstance();
		mAlgaeT = AlgaeT.getInstance();
		mCoralPivot = CoralPivot.getInstance();
		mCoralRoller = CoralRoller.getInstance();
		mElevator = Elevator.getInstance();
		//mClimber = Climber.getInstance();


		autoChooser.setDefaultOption("Just Forward", new justForwardC());
		autoChooser.addOption("SimpleForwardC", new simpleForwardC());
		autoChooser.addOption("L1Attempt", new L1Attempt());
		autoChooser.addOption("Left1", new Left1());
		autoChooser.addOption("Left2", new Left2());
		autoChooser.addOption("LAlgae2", new LAlgae2());
		
		if(isReal()){
			Pose2d startC = Pose2d.fromLegacy(Choreo.loadTrajectory("SimpleForward").get().getInitialPose(is_red_alliance).get());
			mDrive.resetOdometry(startC);
			mDrive.zeroGyro(startC.getRotation().getDegrees());
		}
		
		//autoChooser.addOption("Center 6", new AmpRaceAuto(drivetrain, vision, shooter, shooterPivot, intake, intakePivot, false, 5, 4, 3, 2));
		SmartDashboard.putData("Auto Mode", autoChooser);

		SmartDashboard.putData("Auto Chosen", autoChooser);
	}

		

	@Override
	public void robotInit() {
		try {

			leds.strobeGreen(0.2);
			mDrive = Drive.getInstance();
			mCancoders = Cancoders.getInstance();
			CrashTracker.logRobotInit();

			LiveWindow.disableAllTelemetry();

			SmartDashboard.putBoolean("Is Comp", Constants1678.isComp);
			SmartDashboard.putBoolean("Is Epsilon", Constants1678.isEpsilon);
			SmartDashboard.putString("Serial Number", serial);

			if (!Constants1678.isComp && !Constants1678.isEpsilon) {
				SmartDashboard.putString("Comp Serial", Constants1678.kCompSerial);
				SmartDashboard.putString("Epsilon Serial", Constants1678.kEpsilonSerial);
				SmartDashboard.putString("Serial Number", serial);
			}

			if (Robot.isReal()) {
				mCancoders = Cancoders.getInstance();
				double startInitTs = Timer.getFPGATimestamp();
				System.out.println("* Starting to init Cancoders at ts " + startInitTs);
				while (Timer.getFPGATimestamp() - startInitTs < Constants1678.SwerveConstants.kCancoderBootAllowanceSeconds
						&& !mCancoders.allHaveBeenInitialized()) {
					Timer.delay(0.1);
				}
				System.out.println(
						"* Cancoders all initialized: Took " + (Timer.getFPGATimestamp() - startInitTs) + " seconds");
			}

			mDrive.resetModulesToAbsolute();

			// spotless:off
			mSubsystemManager.setSubsystems(
				mDrive, 
				mSuperstructure,
				mAlgaeRoller,
				mCoralPivot,
				mElevator,
				mCoralRoller,
				mAlgaeT

			);
			// spotless:on

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			TrajectoryGenerator.getInstance().generateTrajectories();
			RobotState.getInstance().resetKalman();
			mDrive.setNeutralBrake(true);

			RobotController.setBrownoutVoltage(5.5);

			DataLogManager.start();

			mSuperstructure.showLevel();
			mSuperstructure.showAngle();


		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void robotPeriodic() {
		mEnabledLooper.outputToSmartDashboard();
		mSubsystemManager.outputLoopTimes();
	}

	private final SendableChooser<AutoModeBase> autoChooser = new SendableChooser<AutoModeBase>();

	AutoFactory autoFactory;

	@Override
	public void autonomousInit() {
		//if (mVisionDevices.getMovingAverageRead() != null) {
		//	mDrive.zeroGyro(mVisionDevices.getMovingAverageRead());
		//}
		RobotState.getInstance().setIsInAuto(true);
		mDisabledLooper.stop();
		mEnabledLooper.start();
		mAutoModeExecutor.setAutoMode(autoChooser.getSelected());//autoChooser.getSelected());
		mAutoModeExecutor.start();

		

		//autoFactory = Choreo.createAutoFactory(mDrive,
		//mDrive::getLegacyPose,
		//mDrive::choreoController,
		//is_red_alliance,
		//new AutoBindings()//,mDrive::choreoLogger	
		//);
		/*
		new SequentialCommandGroup(
			new InstantCommand(() -> mDrive.feedTeleopSetpointFromLegacy(new edu.wpi.first.math.kinematics.ChassisSpeeds(1.0, 1.0, 0.0))),
			new WaitCommand(1.5),
			new InstantCommand(() -> mDrive.feedTeleopSetpointFromLegacy(new edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.0)))
		).schedule();
		 */
	}

	@Override
	public void autonomousPeriodic() {

		 
	}

	@Override
	public void autonomousExit() {
		mDrive.feedTeleopSetpoint(new ChassisSpeeds(0.0, 0.0, 0.0));
	}

	@Override
	public void teleopInit() {
		try {
			RobotState.getInstance().setIsInAuto(false);
			mDrive.feedTeleopSetpoint(new ChassisSpeeds(0.0,  0.0, 0.0));
			VisionDeviceManager.setDisableVision(false);
			mDisabledLooper.stop();
			mEnabledLooper.start();

			//mLimelight.setPipeline(Pipeline.TELEOP);
			//mCoralPivot.zeroSensors();
			//mCoralPivot.setWantHome(true);
				
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopPeriodic() {
		mDriverControls.twoControllerMode();
		try {

			mControlBoard.update();

			/* Drive */
			if (mControlBoard.zeroGyro()) {
				System.out.println("Zeroing gyro!");
				mDrive.zeroGyro(FieldLayout.handleAllianceFlip(new Rotation2d(), is_red_alliance)
						.getDegrees());
				mDrive.resetModulesToAbsolute();
				mDrive.resetOdometry(new Pose2d());
			}

			mDrive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(
					mControlBoard.getSwerveTranslation().x(),
					mControlBoard.getSwerveTranslation().y(),
					mControlBoard.getSwerveRotation(),
					Util.robotToFieldRelative(mDrive.getHeading(), is_red_alliance)));

			ShuffleboardTab coralTab = Shuffleboard.getTab("Coral");
			//coralTab.add("Position", 3);
			//coralTab.add("Level", 3);
			//coralTab.add("Slot", 3);

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		try {
			VisionDeviceManager.setDisableVision(false);
			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();
			mDisabledLooper.start();
			//mCoralPivot.setOpenLoop(0);
			disable_enter_time = Timer.getFPGATimestamp();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

		if (mAutoModeExecutor != null) {
			mAutoModeExecutor.stop();
		}

		//mLimelight.setPipeline(is_red_alliance ? Pipeline.AUTO_RED : Pipeline.AUTO_BLUE);
	}

	@Override
	public void disabledPeriodic() {
		try {
			boolean alliance_changed = false;
			if (DriverStation.getAlliance().isPresent()) {
				if (DriverStation.getAlliance().get() == Alliance.Red) {
					alliance_changed = !is_red_alliance;
					is_red_alliance = true;
				} else if (DriverStation.getAlliance().get() == Alliance.Blue) {
					alliance_changed = is_red_alliance;
					is_red_alliance = false;
				}
			} else {
				alliance_changed = true;
			}

			if (Timer.getFPGATimestamp() - disable_enter_time > 5.0) {
				disable_enter_time = Double.POSITIVE_INFINITY;
			}

			if (alliance_changed) {
				System.out.println("Alliance changed! Requesting trajectory regeneration!");
				TrajectoryGenerator1678.getInstance().forceRegenerateTrajectories(is_red_alliance);
				//mLimelight.setPipeline(is_red_alliance ? Pipeline.AUTO_RED : Pipeline.AUTO_BLUE);
			}

			//mAutoModeSelector.updateModeCreator(alliance_changed);

			//if (autoMode.isPresent() && (autoMode.get() != mAutoModeExecutor.getAutoMode())) {
			//	mAutoModeExecutor.setAutoMode(autoMode.get());
			//}

			//List<Trajectory<TimedState<Pose2dWithMotion>>> paths =
					//autoMode.get().getPaths();
			//for (int i = 0; i < paths.size(); i++) {
				//LogUtil.recordTrajectory(String.format("Paths/Path %d", i), paths.get(i));
			//}

			if (mControlBoard.driver.getBButton()) {
				RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
			}
			//Logger.recordOutput("Vision Heading/Average", mVisionDevices.getMovingAverageRead());

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {
		try {
			mDisabledLooper.stop();
			mEnabledLooper.stop();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testPeriodic() {}
}
