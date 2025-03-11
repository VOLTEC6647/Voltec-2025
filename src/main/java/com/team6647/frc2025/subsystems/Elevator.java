package com.team6647.frc2025.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem;
import com.team1678.lib.requests.Request;
import com.team1678.lib.util.DelayedBoolean;
import com.team254.lib.util.Util;
import com.team6647.frc2025.Constants;
import com.team6647.frc2025.Constants.ElevatorConstants;
import com.team6647.frc2025.subsystems.Superstructure.Levels;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends ServoMotorSubsystem {
	// Singleton instance
	public static Elevator mInstance;

	public static Elevator getInstance() {
		if (mInstance == null) {
			mInstance = new Elevator(ElevatorConstants.kElevatorServoConstants);
		}
		return mInstance;
	}

	private Elevator(final ServoMotorSubsystemConstants constants) {
		super(constants);
		zeroSensors();
		enableSoftLimits(false);
	}

	public static final double kL1Height = 0.0;
	public static final double kL2Height = 0.0;
	public static final double kL3Height = 0.14;
	public static final double kL4Height = 0.39;//0.31;

	public static final double kAlgaeing1 = 0.0;
	public static final double kAlgaeing2 = 0.10;

	private boolean mHoming = false;
	private boolean mNeedsToHome = false;
	private final DelayedBoolean mHomingDelay =
			new DelayedBoolean(Timer.getFPGATimestamp(), Constants.ElevatorConstants.kHomingTimeout);

	@Override
	public void registerEnabledLoops(ILooper mEnabledLooper) {
		mEnabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				setOpenLoop(0.0);
			}

			@Override
			public void onLoop(double timestamp) {
				// Home if we're ready to home
				if (getSetpoint() == mConstants.kHomePosition && atHomingLocation() && mNeedsToHome && !mHoming) {
					setWantHome(true);
					// If we're done homing, we no longer need to home
				} else if (mControlState != ControlState.OPEN_LOOP && mHoming) {
					setWantHome(false);
				}
			}

			@Override
			public void onStop(double timestamp) {}
		});
	}

	public void setWantHome(boolean home) {
		mHoming = home;
		if (home) {
			mNeedsToHome = false;
			mHomingDelay.update(Timer.getFPGATimestamp(), false);
		}
	}
	private double homingOutput;

	@Override
	public void writePeriodicOutputs() {
		if (mHoming) {
			if (mPeriodicIO.position_units>0.4){
				homingOutput = Constants.ElevatorConstants.kMaxHomingOutput;
			}else{
			homingOutput = Constants.ElevatorConstants.kHomingOutput;
			}
			setOpenLoop(homingOutput / mConstants.kMaxForwardOutput);
			if (mHomingDelay.update(
					Timer.getFPGATimestamp(),
					Math.abs(getVelocity()) < Constants.ElevatorConstants.kHomingVelocityWindow)) {
				zeroSensors();
				setSetpointMotionMagic(mConstants.kHomePosition);
				setWantHome(false);
				mNeedsToHome = false;
			}
		}

		super.writePeriodicOutputs();
	}

	@Override
	public void stop() {
		super.stop();
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public boolean atHomingLocation() {
		return mPeriodicIO.position_units - mConstants.kHomePosition < Constants.ElevatorConstants.kHomingZone;
	}

	@Override
	public synchronized void outputTelemetry() {
		Logger.recordOutput(mConstants.kName + "/Homing", mHoming);
		Logger.recordOutput(mConstants.kName + "/Within Homing Window", atHomingLocation());
		Logger.recordOutput(mConstants.kName + "/Setpoint", getSetpoint());

		super.outputTelemetry();
	}

	/**
	 * @return New reqeust commanding the elevator to extend for Trap scoring.
	 */
	public Request L2Request() {
		return new Request() {

			@Override
			public void act() {
				setSetpointMotionMagic(kL2Height);
				mNeedsToHome = true;
			}

			@Override
			public boolean isFinished() {
				return trajectoryDone();
			}
		};
	}

	public Request LRequest(Levels level) {
		return new Request() {

			@Override
			public void act() {
				setSetpointMotionMagic(level.elevatorHeight);
				mNeedsToHome = true;
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), level.elevatorHeight, 0.1);
			}
		};
	}

	
	/**
	 * @return New reqeust commanding the elevator to extend for Amp scoring.
	 */
	public Request L3Request() {
		return new Request() {

			@Override
			public void act() {
				setSetpointMotionMagic(kL3Height);
				mNeedsToHome = true;
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), kL3Height, 0.1);
			}
		};
	}
}