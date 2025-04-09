// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6647.frc2025;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.team1678.frc2024.Constants1678;
import com.team1678.frc2024.SubsystemManager;
import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeExecutor;
import com.team6647.frc2025.auto.AutoModeSelector;

import com.team6647.frc2025.auto.modes.configuredQuals.L4AutoPP;
import com.team6647.frc2025.auto.modes.configuredQuals.S3RightPP;
import com.team6647.frc2025.auto.modes.configuredQuals.simpleForwardD;
import com.team1678.frc2024.controlboard.ControlBoard;
import com.team6647.frc2025.controlboard.DriverControls;
import com.team1678.frc2024.loops.CrashTracker;
import com.team1678.frc2024.loops.Looper;
import com.team1678.frc2024.subsystems.AlgaeT;
import com.team1678.frc2024.subsystems.Climber;
import com.team1678.frc2024.subsystems.CoralPivot;
import com.team1678.frc2024.subsystems.SubsystemV;
import com.team4678.CommandSwerveDrivetrain;
import com.team4678.TunerConstants;
import com.team6647.frc2025.subsystems.Elevator;
import com.team6647.frc2025.subsystems.MotorTest;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.algae_roller.AlgaeRoller;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;
import com.team6647.frc2025.subsystems.leds.LEDSubsystem;
import com.team6647.frc2025.subsystems.vision.VisionSubsystem;
import com.team6647.lib.util.QuestNav;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.frc2025.Constants6328;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.commands.DriveCommands;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.drive.GyroIO;
import org.littletonrobotics.frc2025.subsystems.drive.GyroIOPigeon2;
import org.littletonrobotics.frc2025.subsystems.drive.ModuleIOComp;
import org.littletonrobotics.frc2025.subsystems.drive.ModuleIOSim;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.IOException;
import java.util.function.Supplier;

import com.team6647.frc2025.auto.modes.configuredQuals.TunePP;

public class Robot extends LoggedRobot {

	// util instances
	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
	private SubsystemV[] mSubsystems;
	private final ControlBoard mControlBoard = ControlBoard.getInstance();
	private final DriverControls mDriverControls = new DriverControls();

	// the boss
	private final Superstructure mSuperstructure = Superstructure.getInstance();

	// subsystem instances
	private CommandSwerveDrivetrain mDrive;
	private final SwerveRequest.FieldCentric driveC = new SwerveRequest.FieldCentric();
	private LEDSubsystem leds;

	private MotorTest mMotorTest;
	private AlgaeRoller mAlgaeRoller;
	private AlgaeT mAlgaeT;
	private CoralPivot mCoralPivot;
	private CoralRoller mCoralRoller;

	private Elevator mElevator;
	private com.team1678.frc2024.subsystems.Climber mClimber;
	private VisionSubsystem mVision;

	// vision
	// private final VisionDeviceManager mVisionDevices =
	// VisionDeviceManager.getInstance();

	// limelight
	// private final Limelight mLimelight = Limelight.getInstance();

	// enabled and disabled loopers
	private final Looper mEnabledLooper = new Looper();
	private final Looper mDisabledLooper = new Looper();

	// auto instances
	private AutoModeExecutor mAutoModeExecutor = new AutoModeExecutor();
	// public static final AutoModeSelector mAutoModeSelector = new
	// AutoModeSelector();
	public static final AutoModeSelector mThreeNoteNoteSelector = new AutoModeSelector();
	public static boolean is_red_alliance = false;
	public static boolean is_event = false;
	public static String serial;

	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

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
			Logger.addDataReceiver(new NT4Publisher());
			// setUseTiming(false); // Run as fast as possible
			// String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from
			// AdvantageScope (or prompt the user)
			// Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
			// Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
			// "_sim")));
		}
		Logger.start();
		Constants1678.isComp = serial.startsWith(Constants1678.kCompSerial);
		Constants1678.isEpsilon = serial.startsWith(Constants1678.kEpsilonSerial);
	}

	public Robot() {
		CrashTracker.logRobotConstruction();

		mDrive = TunerConstants.createDrivetrain();

		switch (Constants6328.getRobot()) {
			case COMPBOT -> {
			}
			case DEVBOT -> {

			}
			case SIMBOT -> {
			
			}
		}

		leds = LEDSubsystem.getInstance();

		// mMotorTest = MotorTest.getInstance();
		mAlgaeRoller = AlgaeRoller.getInstance();
		mAlgaeT = AlgaeT.getInstance();
		mCoralPivot = CoralPivot.getInstance();
		mCoralRoller = CoralRoller.getInstance();
		mElevator = Elevator.getInstance();
		mClimber = Climber.getInstance();
		mVision = VisionSubsystem.getInstance();

		autoChooser.addOption("Just Forward", new simpleForwardD());
		autoChooser.addOption("S3RightPP", new S3RightPP());
		autoChooser.addOption("L4", new L4AutoPP());
		autoChooser.addOption("Tuning", new TunePP());

		if (isReal()) {
			RobotState state = RobotState.getInstance();
			Pose2d autoPose;
			try {
				autoPose = FieldLayout.handleAllianceFlip(
						PathPlannerPath.fromPathFile("SF").getStartingHolonomicPose().get(),
						DriverStation.getAlliance().get() == Alliance.Red);
				// state.resetGyro(autoPose.getRotation());
				// state.resetPose(autoPose);
				 QuestNav.getInstance().setPosition(autoPose);
			} catch (FileVersionException e) {
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			} catch (ParseException e) {
				e.printStackTrace();
			}

		}

		// autoChooser.addOption("Center 6", new AmpRaceAuto(drivetrain, vision,
		// shooter, shooterPivot, intake, intakePivot, false, 5, 4, 3, 2));
		SmartDashboard.putData("Auto Mode", autoChooser);

		SmartDashboard.putData("Auto Chosen", autoChooser);
	}

	@Override
	public void robotInit() {
		try {
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

			//mDrive.resetHeadings();

			// mSubsystems = new SubsystemV[]{
			// mDrive
			// };
			// spotless:off
			mSubsystemManager.setSubsystems(
					mSuperstructure,
					mAlgaeRoller,
					mCoralPivot,
					mElevator,
					mCoralRoller,
					mAlgaeT,
					mClimber// ,
			// mVision
			);
			// spotless:on
			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			RobotController.setBrownoutVoltage(6.0);

			DataLogManager.start();

			mSuperstructure.showLevel();
			mSuperstructure.showAngle();

			leds.solidBlue();
			// leds.escuderia_effect();
			PathfindingCommand.warmupCommand().schedule();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
		 CommandScheduler.getInstance().enable();
		confugureButtonBindings();
	}

	@Override
	public void robotPeriodic() {
		mEnabledLooper.outputToSmartDashboard();
		mSubsystemManager.outputLoopTimes();
		CommandScheduler.getInstance().run();
		QuestNav.getInstance().processHeartbeat();
		QuestNav.getInstance().cleanUpQuestNavMessages();
	}

	private final SendableChooser<AutoModeBase> autoChooser = new SendableChooser<AutoModeBase>();

	AutoFactory autoFactory;

	@Override
	public void autonomousInit() {
		// if (mVisionDevices.getMovingAverageRead() != null) {
		// mDrive.zeroGyro(mVisionDevices.getMovingAverageRead());
		// }
		// for (SubsystemV s:mSubsystems){
		// s.onStart(Timer.getFPGATimestamp());
		// }
		mDisabledLooper.stop();
		mEnabledLooper.start();
		mAutoModeExecutor.setAutoMode(autoChooser.getSelected());// autoChooser.getSelected());
		mAutoModeExecutor.start();
	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void autonomousExit() {
		mDrive.stopDrive();
	}

	@Override
	public void teleopInit() {
		try {
			mDrive.stopDrive();
			// VisionDeviceManager.setDisableVision(false);
			// for (SubsystemV s:mSubsystems){
			// s.onStart(Timer.getFPGATimestamp());
			// }
			mDisabledLooper.stop();
			mEnabledLooper.start();

			if(VisionSubsystem.getInstance().coralLimelight.getTagArea()!=0){
				RobotState.getInstance().resetPose(mVision.bestPose);
			QuestNav.getInstance().setPosition(mVision.getBestPose());
			}
			

			// mLimelight.setPipeline(Pipeline.TELEOP);
			// mCoralPivot.zeroSensors();
			// mCoralPivot.setWantHome(true);

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

			ShuffleboardTab coralTab = Shuffleboard.getTab("Coral");
			// coralTab.add("Position", 3);
			// coralTab.add("Level", 3);
			// coralTab.add("Slot", 3);
			mSuperstructure.showAngle();
			mSuperstructure.showLevel();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		try {
			// VisionDeviceManager.setDisableVision(false);
			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();
			mDisabledLooper.start();
			// for (SubsystemV s:mSubsystems){
			// s.onStop(Timer.getFPGATimestamp());
			// }

			// mCoralPivot.setOpenLoop(0);
			disable_enter_time = Timer.getFPGATimestamp();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

		if (mAutoModeExecutor != null) {
			mAutoModeExecutor.stop();
		}

		// mLimelight.setPipeline(is_red_alliance ? Pipeline.AUTO_RED :
		// Pipeline.AUTO_BLUE);
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
			if (DriverStation.getMatchType() != MatchType.None) {
				is_event = true;
			}

			// Timer.getFPGATimestamp()
			if (Timer.getFPGATimestamp() - disable_enter_time > 5.0) {
				disable_enter_time = Double.POSITIVE_INFINITY;
			}

			if (alliance_changed) {
				System.out.println("Alliance changed! But that doesn't matter :/");
			}

			// mAutoModeSelector.updateModeCreator(alliance_changed);

			// if (autoMode.isPresent() && (autoMode.get() !=
			// mAutoModeExecutor.getAutoMode())) {
			// mAutoModeExecutor.setAutoMode(autoMode.get());
			// }

			// List<Trajectory<TimedState<Pose2dWithMotion>>> paths =
			// autoMode.get().getPaths();
			// for (int i = 0; i < paths.size(); i++) {
			// LogUtil.recordTrajectory(String.format("Paths/Path %d", i), paths.get(i));
			// }

			// if (mControlBoard.driver.getBButton()) {
			// RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
			// }
			// Logger.recordOutput("Vision Heading/Average",
			// mVisionDevices.getMovingAverageRead());

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {
		try {
			// for (SubsystemV s:mSubsystems){
			// s.onStop(Timer.getFPGATimestamp());
			// }
			mDisabledLooper.stop();
			mEnabledLooper.stop();
			// for (SubsystemV s:mSubsystems){
			// s.onStop(Timer.getFPGATimestamp());
			// }
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testPeriodic() {
	}

	public static boolean isJITing() {
		return Timer.getTimestamp() < 45.0; // Check if <45 seconds since robot boot
	}

	private void confugureButtonBindings() {

		mDrive.setDefaultCommand(
            // Drivetrain will execute this command periodically
            mDrive.applyRequest(() ->
				driveC.withVelocityX(-mControlBoard.driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-mControlBoard.driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-mControlBoard.driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

		// new ParallelRaceGroup(

		// new WaitUntilCommand(()->DriverStation.isEnabled())
		// ).schedule();

	}
}
