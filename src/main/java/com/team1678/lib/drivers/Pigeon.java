package com.team1678.lib.drivers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.team1678.frc2024.Constants1678;
import com.team1678.frc2024.Ports1678;
import com.team254.lib.geometry.Rotation2d;
import com.team6647.frc2025.Constants.DriveConstants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class Pigeon {

	private static Pigeon mInstance;

	public static Pigeon getInstance() {
		if (mInstance == null) {
			mInstance = new Pigeon(Ports1678.PIGEON);
		}
		return mInstance;
	}

	// Actual pigeon object
	private final Pigeon2 mGyro;

	// Configs
	private boolean inverted = Constants1678.SwerveConstants.invertGyro;
	private Rotation2d yawAdjustmentAngle = new Rotation2d();
	private Rotation2d rollAdjustmentAngle = new Rotation2d();
	private Rotation2d pitchAdjustmentAngle = new Rotation2d();

	private Pigeon(int port) {
		mGyro = new Pigeon2(port, DriveConstants.swerveCANBus);
		mGyro.getConfigurator().apply(new Pigeon2Configuration());
	}

	public Rotation2d getYaw() {
		Rotation2d angle = getUnadjustedYaw().rotateBy(yawAdjustmentAngle.inverse());
		if (inverted) {
			return angle.inverse();
		}
		return angle;
	}

	public Rotation2d getRoll() {
		return getUnadjustedRoll().rotateBy(rollAdjustmentAngle.inverse());
	}

	public Rotation2d getPitch() {
		return getUnadjustedPitch().rotateBy(pitchAdjustmentAngle.inverse()).inverse();
	}

	/**
	 * Sets the yaw register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setYaw(double angleDeg) {
		yawAdjustmentAngle = Rotation2d.fromDegrees(getYawStatusSignal().getValueAsDouble())
				.rotateBy(Rotation2d.fromDegrees(angleDeg).inverse());
	}

	/**
	 * Sets the roll register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setRoll(double angleDeg) {
		rollAdjustmentAngle =
				getUnadjustedRoll().rotateBy(Rotation2d.fromDegrees(angleDeg).inverse());
	}

	/**
	 * Sets the roll register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setPitch(double angleDeg) {
		pitchAdjustmentAngle =
				getUnadjustedPitch().rotateBy(Rotation2d.fromDegrees(angleDeg).inverse());
		System.out.println("Reset gyro to " + getPitch().getDegrees());
	}

	public Rotation2d getUnadjustedYaw() {
		return Rotation2d.fromDegrees(
				BaseStatusSignal.getLatencyCompensatedValueAsDouble(getYawStatusSignal(), getRateStatusSignal()));
	}

	public Rotation2d getUnadjustedPitch() {
		return Rotation2d.fromDegrees(mGyro.getRoll().getValue().baseUnitMagnitude());
	}

	public Rotation2d getUnadjustedRoll() {
		return Rotation2d.fromDegrees(mGyro.getPitch().getValue().baseUnitMagnitude());
	}

	public StatusSignal<Angle> getYawStatusSignal() {
		StatusSignal<Angle> angleSignal = mGyro.getYaw();
		//return new StatusSignal<Double>(angleSignal.getClass(), angleSignal.asSupplier().get().baseUnit(), angleSignal.);
		return angleSignal;
	}

	public StatusSignal<Angle> getPitchStatusSignal() {
		StatusSignal<Angle> angleSignal = mGyro.getPitch();
		//return new StatusSignal<Double>(angleSignal.getClass(), angleSignal.asSupplier().get().baseUnit(), angleSignal.);
		return angleSignal;
	}

	public StatusSignal<Angle> getRollStatusSignal() {
		StatusSignal<Angle> angleSignal = mGyro.getRoll();
		//return new StatusSignal<Double>(angleSignal.getClass(), angleSignal.asSupplier().get().baseUnit(), angleSignal.);
		return angleSignal;
	}

	public StatusSignal<AngularVelocity> getRateStatusSignal() {
		return mGyro.getAngularVelocityZDevice();
	}

	public StatusSignal<AngularVelocity> getAV() {
		return mGyro.getAngularVelocityZWorld();
	}

	public StatusSignal<AngularVelocity> getAVZ() {
		return mGyro.getAngularVelocityZWorld();
	}

	public StatusSignal<AngularVelocity> getAVX() {
		return mGyro.getAngularVelocityXWorld();
	}

	public StatusSignal<AngularVelocity> getAVY() {
		return mGyro.getAngularVelocityYWorld();
	}
}
