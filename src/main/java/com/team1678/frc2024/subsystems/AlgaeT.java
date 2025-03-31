package com.team1678.frc2024.subsystems;



import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem;

import com.team1678.lib.util.Stopwatch;

import com.team6647.frc2025.Constants.AlgaeTConstants;
import com.team6647.frc2025.Constants.CoralPivotConstants;

import com.team6647.frc2025.Ports;


public class AlgaeT extends ServoMotorSubsystem {
	private static AlgaeT mInstance;
	private boolean mHoming = false;
	private Stopwatch mHomingStart = new Stopwatch();
	public static final double kIdleAngle = 0.0, kIntakingAngle = 65.0, kHoldingAngle = 25.0;

	public static AlgaeT getInstance() {
		if (mInstance == null) {
			mInstance = new AlgaeT(
				AlgaeTConstants.kHoodServoConstants);//, CoralPivotConstants.kHoodEncoderConstants
		}
		return mInstance;
	}

	private AlgaeT(ServoMotorSubsystemConstants constants) {//, final AbsoluteEncoderConstants encoder_constants
		super(constants);//, encoder_constants
		//zeroSensors();
		changeTalonConfig((conf) -> {
			conf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
			//conf.Feedback.FeedbackRemoteSensorID = Ports.CLIMBER.getDeviceNumber();
			conf.Feedback.RotorToSensorRatio = (CoralPivotConstants.kRotorRotationsPerOutputRotation)
					/ (mConstants.kRotationsPerUnitDistance * 360.0);
			conf.Feedback.SensorToMechanismRatio = 1.0;
			return conf;
		});
		enableSoftLimits(false);
	}

	@Override
	public void registerEnabledLoops(ILooper mEnabledLooper) {
		mEnabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				setSetpointMotionMagic(0.0);
			}

			@Override
			public void onLoop(double timestamp) {}

			@Override
			public void onStop(double timestamp) {
				setNeutralMode(NeutralModeValue.Brake);
			}
		});
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		super.writePeriodicOutputs();
	}

	@Override
	public synchronized void outputTelemetry() {
		super.outputTelemetry();
	}

	public void fast(){
		
	}

	
}