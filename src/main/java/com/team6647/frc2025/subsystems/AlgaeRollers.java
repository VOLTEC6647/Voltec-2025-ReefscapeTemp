package com.team6647.frc2025.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.Subsystem;
import com.team1678.lib.requests.Request;
import com.team254.lib.drivers.TalonUtil;
import com.team6647.frc2025.Constants.AlgaeRollerConstants;
import com.team6647.frc2025.Ports;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeRollers extends Subsystem {
	private static AlgaeRollers mInstance;

	public static AlgaeRollers getInstance() {
		if (mInstance == null) {
			mInstance = new AlgaeRollers();
		}
		return mInstance;
	}

	public enum State {
		IDLE(0.0),
		INTAKING(1.5),
		EXHAUST(-1.5);

		public double roller_voltage;

		State(double roller_voltage) {
			this.roller_voltage = roller_voltage;
		}
	}

	private final TalonFX mRoller1;
    private final TalonFX mRoller2;

	private State mState = State.IDLE;
	private PeriodicIO mPeriodicIO = new PeriodicIO();

	private AlgaeRollers() {
        //mRoller = new SparkMax(com.team6647.frc2025.Ports.ALGAE_ROLLER.getDeviceNumber(), MotorType.kBrushless);
		mRoller1 = new TalonFX(Ports.ALGAE_ROLLER1.getDeviceNumber(), Ports.ALGAE_ROLLER1.getBus());
		TalonUtil.applyAndCheckConfiguration(mRoller1, AlgaeRollerConstants.RollerFXConfig());
		mRoller1.setInverted(false);

        mRoller2 = new TalonFX(Ports.ALGAE_ROLLER2.getDeviceNumber(), Ports.ALGAE_ROLLER2.getBus());
		TalonUtil.applyAndCheckConfiguration(mRoller2, AlgaeRollerConstants.RollerFXConfig());
		mRoller2.setInverted(false);
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				mPeriodicIO.roller_demand = mState.roller_voltage;
			}

			@Override
			public void onStop(double timestamp) {}
		});
	}

	private static class PeriodicIO implements Sendable {
		// Inputs
		private double roller1_output_voltage;
		private double roller1_stator_current;
		private double roller1_velocity;

        private double roller2_output_voltage;
		private double roller2_stator_current;
		private double roller2_velocity;

		// Outputs
		private double roller_demand;

		@Override
		public void initSendable(SendableBuilder builder) {
			builder.addDoubleProperty("Demand", () -> roller_demand, null);
			builder.addDoubleProperty("VelocityRpS1", () -> roller1_velocity, null);
			builder.addDoubleProperty("OutputVoltage1", () -> roller1_output_voltage, null);
			builder.addDoubleProperty("StatorCurrent1", () -> roller1_stator_current, null);

            builder.addDoubleProperty("VelocityRpS2", () -> roller2_velocity, null);
			builder.addDoubleProperty("OutputVoltage2", () -> roller2_output_voltage, null);
			builder.addDoubleProperty("StatorCurrent2", () -> roller2_stator_current, null);
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
				return mPeriodicIO.roller_demand == _wantedState.roller_voltage;
			}
		};
	}

	@Override
	public void readPeriodicInputs() {
		mPeriodicIO.roller1_output_voltage = mRoller1.getMotorVoltage().getValueAsDouble();
		mPeriodicIO.roller1_stator_current = mRoller1.getStatorCurrent().getValueAsDouble();
		mPeriodicIO.roller1_velocity = mRoller1.getVelocity().getValueAsDouble();

        mPeriodicIO.roller2_output_voltage = mRoller2.getMotorVoltage().getValueAsDouble();
		mPeriodicIO.roller2_stator_current = mRoller2.getStatorCurrent().getValueAsDouble();
		mPeriodicIO.roller2_velocity = mRoller2.getVelocity().getValueAsDouble();
	}

	@Override
	public void writePeriodicOutputs() {
		mRoller1.setControl(new VoltageOut(mPeriodicIO.roller_demand));
        mRoller2.setControl(new VoltageOut(mPeriodicIO.roller_demand));
	}

	@Override
	public void stop() {
		mPeriodicIO.roller_demand = 0.0;
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putString("IntakeRollers/State", mState.toString());
		SmartDashboard.putData("IntakeRollers/IO", mPeriodicIO);
	}
}