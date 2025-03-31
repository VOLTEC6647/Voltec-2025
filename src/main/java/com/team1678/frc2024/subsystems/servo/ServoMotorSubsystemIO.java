package com.team1678.frc2024.subsystems.servo;

import org.littletonrobotics.junction.AutoLog;

import com.team6647.frc2025.subsystems.coral_roller.CoralRollerIO.CoralRollerIOInputs;

public interface ServoMotorSubsystemIO {
    @AutoLog
    public static class ServoMotorSubsystemIOInputs {
        // INPUTS
		public double timestamp;
		public double position_rots; // motor rotations
		public double position_raw; // motor rotations raw
		public double position_units;
		public double velocity_rps;
		public double prev_vel_rps;
		public double output_percent;
		public double output_voltage;
		public double main_stator_current;
		public double main_supply_current;
		public double error_rotations;
		public boolean reset_occured;
		public double active_trajectory_position;
		public double active_trajectory_velocity;
		public double active_trajectory_acceleration;
		public boolean inTolerance;
		public boolean trajectoryDone;

		// OUTPUTS
		public double demand; // position (motor rots) or percent output
		public double demandPID;
    }

    public default void updateInputs(CoralRollerIOInputs inputs) {}

    public default void setVoltage(double voltage) {}
}
