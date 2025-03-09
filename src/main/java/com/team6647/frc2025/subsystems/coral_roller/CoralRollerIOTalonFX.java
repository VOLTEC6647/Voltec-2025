package com.team6647.frc2025.subsystems.coral_roller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team6647.frc2025.Ports;
import com.team6647.frc2025.Constants.CoralRollerConstants;

import edu.wpi.first.math.filter.Debouncer;

import com.playingwithfusion.TimeOfFlight;

public class CoralRollerIOTalonFX implements CoralRollerIO {
    private TalonFX mRoller;
    private TalonFXConfiguration mConfig;
    private TimeOfFlight distanceSensor;
    private Debouncer debounce;

    public CoralRollerIOTalonFX() {
        mRoller = new TalonFX(Ports.CORAL_ROLLER.getDeviceNumber(), Ports.CORAL_ROLLER.getBus());
        mConfig = new TalonFXConfiguration();

        mRoller.setNeutralMode(NeutralModeValue.Brake);
        mConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // this probably fucking broken idk
        mConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        mConfig.CurrentLimits.SupplyCurrentLimit = 30.0f;
        mRoller.getConfigurator().apply(mConfig);
        distanceSensor = new TimeOfFlight(CoralRollerConstants.sensorId);
    }

    @Override
    public void updateInputs(CoralRollerIOInputs inputs) {
        inputs.roller_output_voltage = mRoller.getMotorVoltage().getValueAsDouble();
        inputs.roller_stator_current = mRoller.getStatorCurrent().getValueAsDouble();
        inputs.hasCoral = getBeamBreak();
        inputs.sensorDistance = distanceSensor.getRange();
    }

    @Override
    public void setVoltage(double voltage) {
        mRoller.setControl(new VoltageOut(voltage));
    }

    public boolean getBeamBreak() {
        return distanceSensor.getRange()<CoralRollerConstants.sensorThreshold;
    }
}//0.16342