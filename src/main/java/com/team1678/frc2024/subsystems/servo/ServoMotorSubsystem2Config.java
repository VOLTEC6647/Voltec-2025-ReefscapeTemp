package com.team1678.frc2024.subsystems.servo;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team254.lib.drivers.CanDeviceId;

public class ServoMotorSubsystem2Config {
    public String name = "UNNAMED";
    public CanDeviceId talonCANID;
    public TalonFXConfiguration fxConfig = new TalonFXConfiguration();

    // Ratio of rotor to units for this talon. rotor * by this ratio should
    // be the units.
    // <1 is reduction
    public double unitToRotorRatio = 1.0;
    public double kMinPositionUnits = Double.NEGATIVE_INFINITY;
    public double kMaxPositionUnits = Double.POSITIVE_INFINITY;

    // Moment of Inertia (KgMetersSquared) for sim
    public double momentOfInertia = 0.5;
}