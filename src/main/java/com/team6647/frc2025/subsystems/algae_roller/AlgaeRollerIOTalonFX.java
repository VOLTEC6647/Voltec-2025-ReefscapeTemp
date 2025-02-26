package com.team6647.frc2025.subsystems.algae_roller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team6647.frc2025.Ports;

public class AlgaeRollerIOTalonFX implements AlgaeRollerIO {
    private TalonFX mRoller;
    private TalonFXConfiguration mConfig;

    public AlgaeRollerIOTalonFX() {
        mRoller = new TalonFX(Ports.ALGAE_ROLLER1.getDeviceNumber(), Ports.ALGAE_ROLLER1.getBus());
        mConfig = new TalonFXConfiguration();

        mRoller.setNeutralMode(NeutralModeValue.Brake);
        mConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // this probably fucking broken idk
        mConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        mConfig.CurrentLimits.SupplyCurrentLimit = 30.0f;
        mRoller.getConfigurator().apply(mConfig);
    }

    @Override
    public void updateInputs(AlgaeRollerIOInputs inputs) {
        inputs.roller_output_voltage = mRoller.getMotorVoltage().getValueAsDouble();
        inputs.roller_stator_current = mRoller.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        mRoller.setControl(new VoltageOut(voltage));
    }
}
