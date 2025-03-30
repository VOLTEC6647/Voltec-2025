// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6647.frc2025;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.team1678.frc2024.Constants1678;
import com.team1678.frc2024.RobotState;
import com.team1678.frc2024.SubsystemManager;
import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeExecutor;
import com.team6647.frc2025.auto.AutoModeSelector;

import com.team6647.frc2025.auto.modes.configuredQuals.CTest;
import com.team6647.frc2025.auto.modes.configuredQuals.L1Attempt;
import com.team6647.frc2025.auto.modes.configuredQuals.L4AutoPP;
import com.team6647.frc2025.auto.modes.configuredQuals.LAlgae2;
import com.team6647.frc2025.auto.modes.configuredQuals.Left1;
import com.team6647.frc2025.auto.modes.configuredQuals.Left2;
import com.team6647.frc2025.auto.modes.configuredQuals.Panteras;
import com.team6647.frc2025.auto.modes.configuredQuals.S3Right;
import com.team6647.frc2025.auto.modes.configuredQuals.S3RightA;
import com.team6647.frc2025.auto.modes.configuredQuals.S3RightPP;
import com.team6647.frc2025.auto.modes.configuredQuals.simpleForwardC;
import com.team6647.frc2025.auto.modes.configuredQuals.simpleForwardD;
import com.team1678.frc2024.controlboard.ControlBoard;
import com.team6647.frc2025.controlboard.DriverControls;
import com.team1678.frc2024.loops.CrashTracker;
import com.team1678.frc2024.loops.Looper;
import com.team1678.frc2024.subsystems.AlgaeT;
import com.team1678.frc2024.subsystems.Cancoders;
import com.team1678.frc2024.subsystems.Climber;
import com.team1678.frc2024.subsystems.CoralPivot;
import com.team1678.frc2024.subsystems.SubsystemV;
import com.team6647.frc2025.subsystems.Elevator;
import com.team6647.frc2025.subsystems.MotorTest;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.algae_roller.AlgaeRoller;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;
import com.team6647.frc2025.subsystems.leds.LEDSubsystem;
import com.team6647.frc2025.subsystems.vision.VisionSubsystem;

import choreo.auto.AutoFactory;
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
import org.littletonrobotics.frc2025.Constants6328;
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

import java.util.function.Supplier;

public class Robot extends LoggedRobot {

	// util instances
	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
	private SubsystemV[] mSubsystems;
	private final ControlBoard mControlBoard = ControlBoard.getInstance();
	private final DriverControls mDriverControls = new DriverControls();

	// the boss
	private final Superstructure mSuperstructure = Superstructure.getInstance();

	// subsystem instances
	private Drive mDrive;
	private LEDSubsystem leds;
	private Cancoders mCancoders;

	private MotorTest mMotorTest;
	private AlgaeRoller mAlgaeRoller;
	private AlgaeT mAlgaeT;
	private CoralPivot mCoralPivot;
	private CoralRoller mCoralRoller;

	private Elevator mElevator;
	private com.team1678.frc2024.subsystems.Climber mClimber;
	private VisionSubsystem mVision;





	// vision
	//private final VisionDeviceManager mVisionDevices = VisionDeviceManager.getInstance();

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
	public static boolean is_event = false;
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
			Logger.addDataReceiver(new NT4Publisher());
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

		switch (Constants6328.getRobot()) {
        case COMPBOT -> {
          mDrive =
              Drive.getInstance(
                  new GyroIOPigeon2(),
                  new ModuleIOComp(DriveConstants.moduleConfigsComp[0]),
                  new ModuleIOComp(DriveConstants.moduleConfigsComp[1]),
                  new ModuleIOComp(DriveConstants.moduleConfigsComp[2]),
                  new ModuleIOComp(DriveConstants.moduleConfigsComp[3]));
        }
        case DEVBOT -> {
			
        }
        case SIMBOT -> {
			mDrive =
				Drive.getInstance(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
        }
      }

		leds = LEDSubsystem.getInstance();

		//mMotorTest = MotorTest.getInstance();
		mAlgaeRoller = AlgaeRoller.getInstance();
		mAlgaeT = AlgaeT.getInstance();
		mCoralPivot = CoralPivot.getInstance();
		mCoralRoller = CoralRoller.getInstance();
		mElevator = Elevator.getInstance();
		mClimber = Climber.getInstance();
		mVision = VisionSubsystem.getInstance();
		

		autoChooser.addOption("Just Forward", new simpleForwardD());
		autoChooser.addOption("S3RightA", new S3RightA());
		autoChooser.addOption("SimpleForwardC", new simpleForwardC());
		autoChooser.addOption("L1Attempt", new L1Attempt());
		autoChooser.addOption("Left1", new Left1());
		autoChooser.addOption("Left2", new Left2());
		autoChooser.addOption("LAlgae2", new LAlgae2());
		autoChooser.addOption("CTest", new CTest());
		autoChooser.addOption("S3Right", new S3Right());
		autoChooser.addOption("S3RightPP", new S3RightPP());
		autoChooser.addOption("L4", new L4AutoPP());
		autoChooser.addOption("Panteras", new Panteras());
		
		if(isReal()){

		}
		
		//autoChooser.addOption("Center 6", new AmpRaceAuto(drivetrain, vision, shooter, shooterPivot, intake, intakePivot, false, 5, 4, 3, 2));
		SmartDashboard.putData("Auto Mode", autoChooser);

		SmartDashboard.putData("Auto Chosen", autoChooser);
	}

	@Override
	public void robotInit() {
		try {
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

			mDrive.resetHeadings();

			//mSubsystems = new SubsystemV[]{
			//	mDrive
			//};
			// spotless:off
			mSubsystemManager.setSubsystems(
				mSuperstructure,
				mAlgaeRoller,
				mCoralPivot,
				mElevator,
				mCoralRoller,
				mAlgaeT,
				mClimber//,
				//mVision
			);
			// spotless:on
			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			RobotState.getInstance().resetKalman();

			RobotController.setBrownoutVoltage(6.0);

			DataLogManager.start();

			mSuperstructure.showLevel();
			mSuperstructure.showAngle();

			leds.solidBlue();
			//leds.escuderia_effect();
			PathfindingCommand.warmupCommand().schedule();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
		//CommandScheduler.getInstance().enable();
	}

	@Override
	public void robotPeriodic() {
		mEnabledLooper.outputToSmartDashboard();
		mSubsystemManager.outputLoopTimes();
		CommandScheduler.getInstance().run();
	}

	private final SendableChooser<AutoModeBase> autoChooser = new SendableChooser<AutoModeBase>();

	AutoFactory autoFactory;

	@Override
	public void autonomousInit() {
		//if (mVisionDevices.getMovingAverageRead() != null) {
		//	mDrive.zeroGyro(mVisionDevices.getMovingAverageRead());
		//}
		RobotState.getInstance().setIsInAuto(true);
		for (SubsystemV s:mSubsystems){
			s.onStart(Timer.getFPGATimestamp());
		}
		mDisabledLooper.stop();
		mEnabledLooper.start();
		mAutoModeExecutor.setAutoMode(autoChooser.getSelected());//autoChooser.getSelected());
		mAutoModeExecutor.start();
	}

	@Override
	public void autonomousPeriodic() {

		 
	}

	@Override
	public void autonomousExit() {
		mDrive.stop();
	}

	@Override
	public void teleopInit() {
		try {
			RobotState.getInstance().setIsInAuto(false);
			mDrive.stop();
			//VisionDeviceManager.setDisableVision(false);
			for (SubsystemV s:mSubsystems){
				s.onStart(Timer.getFPGATimestamp());
			}
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

			ShuffleboardTab coralTab = Shuffleboard.getTab("Coral");
			//coralTab.add("Position", 3);
			//coralTab.add("Level", 3);
			//coralTab.add("Slot", 3);
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
			//VisionDeviceManager.setDisableVision(false);
			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();
			mDisabledLooper.start();
			for (SubsystemV s:mSubsystems){
				s.onStop(Timer.getFPGATimestamp());
			}
			
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
			if(DriverStation.getMatchType() != MatchType.None){
				is_event = true;
			}

			//Timer.getFPGATimestamp()
			if (Timer.getFPGATimestamp() - disable_enter_time > 5.0) {
				disable_enter_time = Double.POSITIVE_INFINITY;
			}

			if (alliance_changed) {
				System.out.println("Alliance changed! But that doesn't matter :/");
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

			//if (mControlBoard.driver.getBButton()) {
			//	RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
			//}
			//Logger.recordOutput("Vision Heading/Average", mVisionDevices.getMovingAverageRead());

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {
		try {
			for (SubsystemV s:mSubsystems){
				s.onStop(Timer.getFPGATimestamp());
			}
			mDisabledLooper.stop();
			mEnabledLooper.stop();
			for (SubsystemV s:mSubsystems){
				s.onStop(Timer.getFPGATimestamp());
			}
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testPeriodic() {}

	public static boolean isJITing() {
		return Timer.getTimestamp() < 45.0; // Check if <45 seconds since robot boot
	}

	private void confugureButtonBindings(){
		Supplier<Command> joystickDriveCommandFactory =
		()->DriveCommands.joystickDrive(mDrive, 
		()->mControlBoard.getSwerveTranslation().x(),
		()->mControlBoard.getSwerveTranslation().y(),
		()->mControlBoard.getSwerveRotation(),
		()->is_red_alliance
		//Util.robotToFieldRelative(new Rotation2d(RobotState6328.getInstance().getHeading()),is_red_alliance).toLegacy()
		);
		mDrive.setDefaultCommand(joystickDriveCommandFactory.get());
	}
}
