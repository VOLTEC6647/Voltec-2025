package com.team1678.frc2024.subsystems.servo;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.Subsystem;
import com.team1678.lib.requests.Request;
import com.team1678.lib.swerve.SwerveModule.mPeriodicIO;
import com.team254.lib.drivers.CanDeviceId;
import com.team254.lib.drivers.Phoenix6Util;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.motion.MotionState;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.text.DecimalFormat;
import java.util.function.UnaryOperator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Abstract base class for a subsystem with a single sensored servo-mechanism.
 * spotless:off
 */
public abstract class ServoMotorSubsystemBorePID extends Subsystem {

	// Recommend initializing in a static block!
	public static class TalonFXConstants {
		public CanDeviceId id = new CanDeviceId(-1);
		public boolean counterClockwisePositive = true;
		public boolean invert_sensor_phase = false;
		public int encoder_ppr = 2048;
	}

	// Recommend initializing in a static block!
	public static class ServoMotorSubsystemConstantsBorePID {
		public String kName = "ERROR_ASSIGN_A_NAME";

		public double kLooperDt = 0.01;
		public double kCANTimeout = 0.010; // use for important on the fly updates
		public int kLongCANTimeoutMs = 100; // use for constructors

		public TalonFXConstants kMainConstants = new TalonFXConstants();
		public TalonFXConstants[] kFollowerConstants = new TalonFXConstants[0];

		public NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
		public double kTolerance = 0.0; // Units
		public double kRotationsPerUnitDistance = 1.0;
		public double kSoftLimitDeadband = 0.0;
		public double kKp = 0; // Raw output / raw error
		public double kKi = 0; // Raw output / sum of raw error
		public double kKd = 0; // Raw output / (err - prevErr)

		public int kDeadband = 0; // rotation

		public int kPositionDeadband = 0; // Ticks

		public double kJerk = 0; // Units/s^3
		public double kRampRate = 0.0; // s

		public int kSupplyCurrentLimit = 60; // amps
		public int kSupplyCurrentThreshold = 60;
		public double kSupplyCurrentTimeout = 0.0; // Seconds
		public boolean kEnableSupplyCurrentLimit = false;

		public int kStatorCurrentLimit = 40; // amps
		public boolean kEnableStatorCurrentLimit = false;

		public double kMaxForwardOutput = 12.0; // Volts
		public double kMaxReverseOutput = -12.0; // Volts

		public int kStatusFrame8UpdateRate = 1000;

		public int encoder_id = -1;
	}

	protected final ServoMotorSubsystemConstantsBorePID mConstants;
	protected final TalonFX mMain;
	protected final TalonFX[] mFollowers;

	protected TalonFXConfiguration mMainConfig;
	protected final TalonFXConfiguration[] mFollowerConfigs;

	protected final StatusSignal<Angle> mMainPositionSignal;
	protected final StatusSignal<AngularVelocity> mMainVelocitySignal;
	protected final StatusSignal<Double> mMainClosedLoopError;
	protected final StatusSignal<Current> mMainStatorCurrentSignal;
	protected final StatusSignal<Current> mMainSupplyCurrentSignal;
	protected final StatusSignal<Voltage> mMainOutputVoltageSignal;
	protected final StatusSignal<Double> mMainOutputPercentageSignal;
	protected final StatusSignal<Double> mMainClosedLoopOutputSignal;
	protected final StatusSignal<Double> mMainClosedLoopReferenceSignal;
	protected final StatusSignal<Double> mMainClosedLoopReferenceSlopeSignal;

	protected MotionState mMotionStateSetpoint = null;

	protected double mForwardSoftLimitRotations;
	protected double mReverseSoftLimitRotations;

	protected PIDController pid;
	protected DutyCycleEncoder encoder;

	protected ServoMotorSubsystemBorePID(final ServoMotorSubsystemConstantsBorePID constants) {
		mConstants = constants;
		mMain = TalonFXFactory.createDefaultTalon(mConstants.kMainConstants.id, false);
		mFollowers = new TalonFX[mConstants.kFollowerConstants.length];
		mFollowerConfigs = new TalonFXConfiguration[mConstants.kFollowerConstants.length];

		Phoenix6Util.checkErrorAndRetry(() -> mMain.getBridgeOutput().setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMain.getFault_Hardware().setUpdateFrequency(4, 0.05));

		mMainPositionSignal = mMain.getPosition();
		mMainVelocitySignal = mMain.getVelocity();
		mMainClosedLoopError = mMain.getClosedLoopError();
		mMainStatorCurrentSignal = mMain.getStatorCurrent();
		mMainSupplyCurrentSignal = mMain.getSupplyCurrent();
		mMainOutputVoltageSignal = mMain.getMotorVoltage();
		mMainOutputPercentageSignal = mMain.getDutyCycle();
		mMainClosedLoopReferenceSignal = mMain.getClosedLoopReference();
		mMainClosedLoopOutputSignal = mMain.getClosedLoopOutput();
		mMainClosedLoopReferenceSlopeSignal = mMain.getClosedLoopReferenceSlope();

		Phoenix6Util.checkErrorAndRetry(() -> mMainPositionSignal.setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMainVelocitySignal.setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMainClosedLoopError.setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMainStatorCurrentSignal.setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMainOutputVoltageSignal.setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMainOutputPercentageSignal.setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMainClosedLoopReferenceSignal.setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMainClosedLoopOutputSignal.setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMainClosedLoopReferenceSlopeSignal.setUpdateFrequency(200, 0.05));

		mMainConfig = TalonFXFactory.getDefaultConfig();

		mMainConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = mConstants.kRampRate;
		mMainConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = mConstants.kRampRate;
		mMainConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = mConstants.kRampRate;

		mMainConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = mConstants.kRampRate;
		mMainConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = mConstants.kRampRate;
		mMainConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = mConstants.kRampRate;

		mMainConfig.CurrentLimits.SupplyCurrentLimit = mConstants.kSupplyCurrentLimit+mConstants.kSupplyCurrentThreshold;
		mMainConfig.CurrentLimits.SupplyCurrentLowerLimit = mConstants.kSupplyCurrentLimit-mConstants.kSupplyCurrentThreshold;
		mMainConfig.CurrentLimits.SupplyCurrentLowerTime = mConstants.kSupplyCurrentTimeout; //SUS treshhold to lower
		mMainConfig.CurrentLimits.SupplyCurrentLimitEnable = mConstants.kEnableSupplyCurrentLimit;

		mMainConfig.CurrentLimits.StatorCurrentLimit = mConstants.kStatorCurrentLimit;
		mMainConfig.CurrentLimits.StatorCurrentLimitEnable = mConstants.kEnableStatorCurrentLimit;

		mMainConfig.Voltage.PeakForwardVoltage = mConstants.kMaxForwardOutput;
		mMainConfig.Voltage.PeakReverseVoltage = mConstants.kMaxReverseOutput;

		mMainConfig.MotorOutput.PeakForwardDutyCycle = mConstants.kMaxForwardOutput / 12.0;
		mMainConfig.MotorOutput.PeakReverseDutyCycle = mConstants.kMaxReverseOutput / 12.0;

		mMainConfig.MotorOutput.Inverted = (mConstants.kMainConstants.counterClockwisePositive
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive);

		mMainConfig.MotorOutput.NeutralMode = mConstants.kNeutralMode;

		for (int i = 0; i < mFollowers.length; ++i) {
			mFollowers[i] = TalonFXFactory.createPermanentFollowerTalon(
					mConstants.kFollowerConstants[i].id, mConstants.kMainConstants.id, false);

			TalonFX follower = mFollowers[i];
			mFollowerConfigs[i] = new TalonFXConfiguration();
			TalonFXConfiguration followerConfig = mFollowerConfigs[i];
			Phoenix6Util.checkErrorAndRetry(() -> follower.getConfigurator().refresh(followerConfig));

			followerConfig.MotorOutput.Inverted = (mConstants.kMainConstants.counterClockwisePositive
					? InvertedValue.CounterClockwise_Positive
					: InvertedValue.Clockwise_Positive);
			followerConfig.MotorOutput.NeutralMode = mConstants.kNeutralMode;
			followerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
			followerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
			follower.setControl(new Follower(mConstants.kMainConstants.id.getDeviceNumber(), mConstants.kMainConstants.counterClockwisePositive));

			TalonUtil.applyAndCheckConfiguration(follower, followerConfig);
		}

		TalonUtil.applyAndCheckConfiguration(mMain, mMainConfig);

		pid = new PIDController(mConstants.kKp, mConstants.kKi, mConstants.kKd);
		encoder = new DutyCycleEncoder(mConstants.encoder_id);
		pid.setIntegratorRange(-0.5,0.5);
		// Send a neutral command.
		stop();
	}

	public void setStatorCurrentLimit(double currentLimit, boolean enable) {
		changeTalonConfig((conf) -> {
			conf.CurrentLimits.StatorCurrentLimit = currentLimit;
			conf.CurrentLimits.StatorCurrentLimitEnable = enable;
			return conf;
		});
	}

	public void setNeutralMode(NeutralModeValue mode) {
		changeTalonConfig((conf) -> {
			conf.MotorOutput.NeutralMode = mode;
			return conf;
		});
	}

	public void changeTalonConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
		for (int i = 0; i < mFollowers.length; ++i) {
			mFollowerConfigs[i] = configChanger.apply(mFollowerConfigs[i]);
		}
		mMainConfig = configChanger.apply(mMainConfig);
		writeConfigs();
	}

	public void writeConfigs() {
		for (int i = 0; i < mFollowers.length; ++i) {
			TalonFX follower = mFollowers[i];
			TalonFXConfiguration followerConfig = mFollowerConfigs[i];
			TalonUtil.applyAndCheckConfiguration(follower, followerConfig);
		}
		TalonUtil.applyAndCheckConfiguration(mMain, mMainConfig);
	}

	protected enum ControlState {
		OPEN_LOOP,
		POSITION_PID
	}

	protected ServoMotorSubsystemIOInputsAutoLogged mPeriodicIO = new ServoMotorSubsystemIOInputsAutoLogged();
	protected ControlState mControlState = ControlState.OPEN_LOOP;
	protected boolean mHasBeenZeroed = false;
	protected StatusSignal<Integer> mMainStickyFault;

	@Override
	public synchronized void readPeriodicInputs() {
		mPeriodicIO.timestamp = Timer.getFPGATimestamp();

		if (mMain.hasResetOccurred()) {
			DriverStation.reportError(mConstants.kName + ": Talon Reset! ", false);
			mPeriodicIO.reset_occured = true;
			return;
		} else {
			mPeriodicIO.reset_occured = false;
		}

		mMainStickyFault = mMain.getStickyFaultField();

		if (mMain.getControlMode().getValue() == ControlModeValue.PositionDutyCycle) {
			mPeriodicIO.error_rotations = mMainClosedLoopError.asSupplier().get();
		} else {
			mPeriodicIO.error_rotations = 0;
		}
		mPeriodicIO.error_rotations = mMainClosedLoopError.asSupplier().get();
		mPeriodicIO.main_stator_current = mMainStatorCurrentSignal.asSupplier().get().baseUnitMagnitude();
		mPeriodicIO.main_supply_current = mMainSupplyCurrentSignal.asSupplier().get().baseUnitMagnitude();//SUS
		mPeriodicIO.output_voltage = mMainOutputVoltageSignal.asSupplier().get().baseUnitMagnitude();
		mPeriodicIO.output_percent = mMainOutputPercentageSignal.asSupplier().get();
		mPeriodicIO.position_rots = encoder.get();
		mPeriodicIO.velocity_rps = mMainVelocitySignal.asSupplier().get().baseUnitMagnitude();
		mPeriodicIO.active_trajectory_position =
				mMainClosedLoopReferenceSignal.asSupplier().get();

		mPeriodicIO.inTolerance = inTolerance();
		mPeriodicIO.demandPID = pid.calculate(mPeriodicIO.demand,mPeriodicIO.position_rots);
		Logger.processInputs(mConstants.kName+"/ServoMotor", mPeriodicIO);
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		if (mControlState == ControlState.POSITION_PID) {
			mMain.set(mPeriodicIO.demandPID);
		}
	}

	public synchronized void handleMainReset(boolean reset) {}

	public synchronized void setPosition(double value) {
		mMain.setPosition(value);
	}

	@Override
	public void registerEnabledLoops(ILooper mEnabledLooper) {
		mEnabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				if (mPeriodicIO.reset_occured) {
					System.out.println(mConstants.kName + ": Main Talon reset occurred; resetting frame rates.");
					Phoenix6Util.checkErrorAndRetry(() -> mMainPositionSignal.setUpdateFrequency(200, 0.05));
					Phoenix6Util.checkErrorAndRetry(() -> mMainVelocitySignal.setUpdateFrequency(200, 0.05));
					Phoenix6Util.checkErrorAndRetry(() -> mMainClosedLoopError.setUpdateFrequency(200, 0.05));
					Phoenix6Util.checkErrorAndRetry(() -> mMainStatorCurrentSignal.setUpdateFrequency(200, 0.05));
					Phoenix6Util.checkErrorAndRetry(() -> mMainOutputVoltageSignal.setUpdateFrequency(200, 0.05));
					Phoenix6Util.checkErrorAndRetry(() -> mMainOutputPercentageSignal.setUpdateFrequency(200, 0.05));

					resetIfAtHome();
				}
				handleMainReset(mPeriodicIO.reset_occured);
				for (TalonFX follower : mFollowers) {
					if (follower.hasResetOccurred()) {
						System.out.println(mConstants.kName + ": Follower Talon reset occurred");
					}
				}
			}

			@Override
			public void onStop(double timestamp) {

				stop();
			}
		});
	}

	public synchronized double getPositionRotations() {
		return mPeriodicIO.position_rots;
	}

	// In "Units"
	public synchronized double getPosition() {
		return mPeriodicIO.position_rots;
	}

	// In "Units per second"
	public synchronized double getVelocity() {
		return mPeriodicIO.velocity_rps;
	}

	public synchronized boolean hasFinishedTrajectory() {
		return Util.epsilonEquals(
				mPeriodicIO.active_trajectory_position, getSetpoint(), Math.max(1, mConstants.kDeadband));
	}

	public synchronized double getSetpoint() {
		return mPeriodicIO.demand;
	}

	public synchronized void setSetpointPositionPID(double units) {
		mPeriodicIO.demand = units;
		if (mControlState != ControlState.POSITION_PID) {
			mControlState = ControlState.POSITION_PID;
		}
	}

	protected double rotationsToUnits(double rotations) {
		return rotations / mConstants.kRotationsPerUnitDistance;
	}

	public synchronized void setOpenLoop(double percentage) {
		if (mControlState != ControlState.OPEN_LOOP) {
			mControlState = ControlState.OPEN_LOOP;
		}
		mPeriodicIO.demand = percentage;
	}

	public synchronized String getControlState() {
		return mControlState.toString();
	}

	public synchronized double getPredictedPositionUnits(double lookahead_secs) {
		double predicted_units = mPeriodicIO.active_trajectory_position
				+ lookahead_secs * mPeriodicIO.active_trajectory_velocity
				+ 0.5 * mPeriodicIO.active_trajectory_acceleration * lookahead_secs * lookahead_secs;
		if (mPeriodicIO.demand >= mPeriodicIO.active_trajectory_position) {
			return Math.min(predicted_units, mPeriodicIO.demand);
		} else {
			return Math.max(predicted_units, mPeriodicIO.demand);
		}
	}

	public boolean atHomingLocation() {
		return false;
	}

	public synchronized void resetIfAtHome() {
		if (atHomingLocation()) {
			zeroSensors();
		}
	}

	@Override
	public synchronized void zeroSensors() {
		Phoenix6Util.checkErrorAndRetry(() -> mMain.setPosition(0, mConstants.kCANTimeout));
		mHasBeenZeroed = true;
	}

	public synchronized void forceZero() {
		Phoenix6Util.checkErrorAndRetry(() -> mMain.setPosition(0, mConstants.kCANTimeout));
	}

	public synchronized boolean hasBeenZeroed() {
		return mHasBeenZeroed;
	}

	public synchronized void setSupplyCurrentLimit(double value, boolean enable) {
		mMainConfig.CurrentLimits.SupplyCurrentLimit = value;
		mMainConfig.CurrentLimits.SupplyCurrentLimitEnable = enable;

		TalonUtil.applyAndCheckConfiguration(mMain, mMainConfig);
	}

	public synchronized void setSupplyCurrentLimitUnchecked(double value, boolean enable) {
		mMainConfig.CurrentLimits.SupplyCurrentLimit = value;
		mMainConfig.CurrentLimits.SupplyCurrentLimitEnable = enable;

		mMain.getConfigurator().apply(mMainConfig);
	}

	public synchronized void setStatorCurrentLimitUnchecked(double value, boolean enable) {
		mMainConfig.CurrentLimits.StatorCurrentLimit = value;
		mMainConfig.CurrentLimits.StatorCurrentLimitEnable = enable;

		mMain.getConfigurator().apply(mMainConfig);
	}

	public synchronized void setMotionMagicConfigsUnchecked(double accel, double jerk) {
		mMainConfig.MotionMagic.MotionMagicAcceleration = accel;
		mMainConfig.MotionMagic.MotionMagicJerk = jerk;
		mMain.getConfigurator().apply(mMainConfig.MotionMagic);
	}

	@Override
	public void stop() {
		setOpenLoop(0.0);
		mMain.stopMotor();
	}

	@Override
	public synchronized void outputTelemetry() {
		Logger.recordOutput(mConstants.kName + "/position units", mPeriodicIO.position_units);
		//Logger.recordOutput(mConstants.kName + "/position units2", );
		Logger.recordOutput(mConstants.kName + "/position rots", mPeriodicIO.position_rots);
		Logger.recordOutput(mConstants.kName + "/mode", mMain.getControlMode().toString());

	}

	@Override
	public void rewriteDeviceConfiguration() {
		writeConfigs();
	}

	@Override
	public boolean checkDeviceConfiguration() {
		if (!TalonUtil.readAndVerifyConfiguration(mMain, mMainConfig)) {
			return false;
		}
		for (int i = 0; i < mFollowers.length; ++i) {
			TalonFX follower = mFollowers[i];
			TalonFXConfiguration followerConfig = mFollowerConfigs[i];
			if (!TalonUtil.readAndVerifyConfiguration(follower, followerConfig)) {
				return false;
			}
		}
		return true;
	}

	public synchronized boolean inTolerance() {
		//return Util.epsilonEquals(mPeriodicIO.position_rots-30, getSetpoint(), mConstants.kTolerance);
		//return Util.epsilonEquals(mMainClosedLoopError.asSupplier().get(), getSetpoint(), mConstants.kTolerance);
		return mMainClosedLoopError.asSupplier().get()<mConstants.kTolerance&&mPeriodicIO.demand==mPeriodicIO.active_trajectory_position;
	}

	public static double round2(double value) {
        DecimalFormat df = new DecimalFormat("#.##");
        df.setRoundingMode(java.math.RoundingMode.HALF_UP); // Optional: Set rounding mode
        return Double.parseDouble(df.format(value));
    }

	public synchronized boolean trajectoryDone() {
		return round2(mPeriodicIO.demand)==round2(mPeriodicIO.active_trajectory_position);
	}

	public Request waitRequest() {
    return new Request() {
        @Override
        public void act() {}
        
        @Override
        public boolean isFinished() {
            return trajectoryDone();
        }
    };

	
	}

	public Request setPositionRequest(Double position) {
		return new Request() {

			@Override
			public void act() {
				setSetpointPositionPID(position);
			}

			@Override
			public boolean isFinished() {
				return trajectoryDone();
			}
		};
	}
}
