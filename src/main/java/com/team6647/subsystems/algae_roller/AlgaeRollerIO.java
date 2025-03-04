package com.team6647.subsystems.algae_roller;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeRollerIO {
    @AutoLog
    public static class AlgaeRollerIOInputs {
        public double roller_output_voltage;
        public double roller_stator_current;
        public double roller_demand;
    }

    public default void updateInputs(AlgaeRollerIOInputs inputs) {}

    public default void setVoltage(double voltage) {}
}
