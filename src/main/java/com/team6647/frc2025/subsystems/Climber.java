package com.team6647.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.Subsystem;
import com.team1678.lib.requests.Request;
import com.team6647.frc2025.Ports;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Subsystem {
    private static Climber mInstance;

    public static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    public enum State {
        IDLE(0.0),
        CLIMBING(1.0),
        RETRACTING(-1.0);

        public double climber_voltage;

        State(double climber_voltage) {
            this.climber_voltage = climber_voltage;
        }
    }

    private State mState = State.IDLE;

    private final PeriodicIO mPeriodicIO = new PeriodicIO();
    private final TalonFX mClimber;
    private final TalonFXConfiguration mConfig;

    private Climber() {
        mClimber = new TalonFX(Ports.CLIMBER_LINEAR_ACTUATOR);
        mConfig = new TalonFXConfiguration();
        
        mClimber.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				mPeriodicIO.climber_demand = mState.climber_voltage;
				//.
			}

			@Override
			public void onStop(double timestamp) {}
		});
	}

	private static class PeriodicIO implements Sendable {
		// Inputs
		private double climber_output_voltage;
		private double climber_stator_current;
		private double positionSegment;
		private boolean moving = false;

		// Outputs
		private double climber_demand;

		@Override
		public void initSendable(SendableBuilder builder) {
			builder.addDoubleProperty("Demand", () -> climber_demand, null);
			builder.addDoubleProperty("OutputVoltage", () -> climber_output_voltage, null);
			builder.addDoubleProperty("StatorCurrent", () -> climber_stator_current, null);
		}
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
				return mPeriodicIO.climber_demand == _wantedState.climber_voltage;
			}
		};
	}

	@Override
	public void readPeriodicInputs() {
		mPeriodicIO.climber_output_voltage = mClimber.getMotorOutputStatus().getValueAsDouble();
		mPeriodicIO.climber_stator_current = mClimber.getStatorCurrent().getValueAsDouble();
	}

	@Override
	public void writePeriodicOutputs() {
		//mClimber.setVoltage(mPeriodicIO.climber_demand);
        mClimber.setControl(new VoltageOut(mPeriodicIO.climber_demand));
	}

	@Override
	public void stop() {
		mPeriodicIO.climber_demand = 0.0;
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putString("AlgaeHolder/State", mState.toString());
		SmartDashboard.putData("AlgaeHolder/IO", mPeriodicIO);
	}
}
