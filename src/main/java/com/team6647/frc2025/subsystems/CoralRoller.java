package com.team6647.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.Subsystem;
import com.team1678.lib.requests.Request;
import com.team6647.frc2025.Ports;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralRoller extends Subsystem {
	private static CoralRoller mInstance;

	public static CoralRoller getInstance() {
		if (mInstance == null) {
			mInstance = new CoralRoller();
		}
		return mInstance;
	}

	public enum State {
		IDLE(0.0),
		INTAKING(1.0),
		OUTAKING(-1.0);

		public double holder_voltage;

		State(double holder_voltage) {
			this.holder_voltage = holder_voltage;
		}
	}

	//private final SparkMax mRoller;
	private final TalonFX mRoller;
	private final TalonFXConfiguration mConfig;

	private State mState = State.IDLE;
	private final PeriodicIO mPeriodicIO = new PeriodicIO();

	private CoralRoller() {
		mRoller = new TalonFX(Ports.CORAL_ROLLER.getDeviceNumber());
		mConfig = new TalonFXConfiguration();

		mRoller.setNeutralMode(NeutralModeValue.Brake);
		mConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // this probably fucking broken idk
		mConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		mConfig.CurrentLimits.SupplyCurrentLimit = 30.0f;
		mRoller.getConfigurator().apply(mConfig);
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				mPeriodicIO.holder_demand = mState.holder_voltage;
			}

			@Override
			public void onStop(double timestamp) {}
		});
	}

	private static class PeriodicIO implements Sendable {
		// Inputs
		private double holder_output_voltage;
		private double holder_stator_current;

		// Outputs
		private double holder_demand;

		@Override
		public void initSendable(SendableBuilder builder) {
			builder.addDoubleProperty("Demand", () -> holder_demand, null);
			builder.addDoubleProperty("OutputVoltage", () -> holder_output_voltage, null);
			builder.addDoubleProperty("StatorCurrent", () -> holder_stator_current, null);
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
				return mPeriodicIO.holder_demand == _wantedState.holder_voltage;
			}
		};
	}

	@Override
	public void readPeriodicInputs() {
		mPeriodicIO.holder_output_voltage = mRoller.getMotorOutputStatus().getValueAsDouble(); // maybe we need to check idk
		mPeriodicIO.holder_stator_current = mRoller.getStatorCurrent().getValueAsDouble();
	}

	@Override
	public void writePeriodicOutputs() {
		mRoller.setVoltage(mPeriodicIO.holder_demand);
	}

	@Override
	public void stop() {
		mPeriodicIO.holder_demand = 0.0;
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