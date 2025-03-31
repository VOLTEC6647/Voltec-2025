package com.team6647.frc2025.subsystems.coral_roller;

import org.littletonrobotics.junction.AutoLog;

public interface CoralRollerIO {
    @AutoLog
    public static class CoralRollerIOInputs {
        public double roller_output_voltage;
        public double roller_stator_current;
        public double roller_demand;
        public boolean hasCoral;
        public double sensorDistance;
    }

    public default void updateInputs(CoralRollerIOInputs inputs) {}

    public default void setVoltage(double voltage) {}
}
