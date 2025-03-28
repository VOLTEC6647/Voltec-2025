package com.team1678.frc2024.subsystems;

import java.io.ObjectInputFilter.Config;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.Constants1678.ClimberConstants;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystemWithCancoder;
import com.team1678.lib.requests.Request;
import com.team1678.lib.util.Stopwatch;
import com.team254.lib.util.Util;
import com.team6647.frc2025.Constants;
import com.team6647.frc2025.Constants.CoralPivotConstants;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.Superstructure.Levels;
import com.team6647.frc2025.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends ServoMotorSubsystem {
	private static Climber mInstance;
	private boolean mHoming = false;
	private Stopwatch mHomingStart = new Stopwatch();
	public static final double kPreparing = -134.0, kVertical = 224.0, kIdle = 100.0;

	public static Climber getInstance() {

		if (mInstance == null) {
			mInstance = new Climber(
				com.team6647.frc2025.Constants.ClimberConstants.kHoodServoConstants);//, CoralPivotConstants.kHoodEncoderConstants
		}
		return mInstance;
	}

	private Climber(ServoMotorSubsystemConstants constants) {//, final AbsoluteEncoderConstants encoder_constants
		super(constants);//, encoder_constants
		//zeroSensors();
		changeTalonConfig((conf) -> {
			conf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
			conf.Feedback.FeedbackRemoteSensorID = Ports.CLIMBER.getDeviceNumber();
			conf.Feedback.RotorToSensorRatio = (CoralPivotConstants.kRotorRotationsPerOutputRotation)
					/ (mConstants.kRotationsPerUnitDistance * 360.0);
			conf.Feedback.SensorToMechanismRatio = 1.0;
			return conf;
		});
		enableSoftLimits(false);
		setSetpointMotionMagic(kIdle);
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

	
}