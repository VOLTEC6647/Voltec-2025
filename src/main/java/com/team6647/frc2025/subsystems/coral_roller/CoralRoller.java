package com.team6647.frc2025.subsystems.coral_roller;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.Subsystem;
import com.team1678.lib.requests.Request;
import com.team6647.frc2025.Ports;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralRoller extends Subsystem {
	private static CoralRoller mInstance;
    public static final double OuttakeDelay = 1.5;

	public static CoralRoller getInstance() {
		if (mInstance == null) {
			mInstance = new CoralRoller(new CoralRollerIOTalonFX());
		}
		return mInstance;
	}

	private double kStandard = 1.3;
	public State OUTAKING = State.OUTAKING3;

	public enum State {
		IDLE(0.0),
		INTAKING(-3),//-1.3
		OUTAKING4(1.5),
		OUTAKING3(1.3),
		OUTAKING2(1.3),
		OUTAKING1(3);


		public double roller_demand;

		State(double roller_demand) {
			this.roller_demand = roller_demand;
		}
	}

	private State mState = State.IDLE;
	private CoralRollerIO io;
	private final CoralRollerIOInputsAutoLogged inputs = new CoralRollerIOInputsAutoLogged();

	private CoralRoller(CoralRollerIO io) {
		this.io = io;
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				inputs.roller_demand = mState.roller_demand;
			}

			@Override
			public void onStop(double timestamp) {}
		});
	}

	/**
	 * Gets the current state of the intake rollers.
	 *
	 * @return The current state.
	 */
	public State getState() {
		return mState;
	}

	/**
	 * Sets the state of the intake rollers.
	 *
	 * @param state The state to set.
	 */
	public void setState(State state) {
		mState = state;
	}

	/**
	 * @param _wantedState Wanted state for the intake rollers.
	 * @return New request that updates the intake rollers with the wanted state. 
	 */
	public Request stateRequest(State _wantedState) {
		return new Request() {
			@Override
			public void act() {
				setState(_wantedState);
			}

			@Override
			public boolean isFinished() {
				return inputs.roller_demand == _wantedState.roller_demand;
			}
		};
	}

	@Override
	public void readPeriodicInputs() {
		io.updateInputs(inputs);
	}

	@Override
	public void writePeriodicOutputs() {
		//mRoller.setVoltage(mPeriodicIO.holder_demand);
		//mRoller.setControl(new VoltageOut(mPeriodicIO.holder_demand));
		io.setVoltage(mState.roller_demand);
	}


	@Override
	public void stop() {
		inputs.roller_demand = 0.0;
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public synchronized void outputTelemetry() {
		Logger.recordOutput("CoralRoller/State", mState.toString());
		Logger.processInputs("CoralRoller/IO", inputs);
	}

	public boolean getBeamBreak(){
		return inputs.hasCoral;
	}
}