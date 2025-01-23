package com.team6647.frc2025.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.team1678.frc2024.Constants.AmpRollerConstants;
//import com.team1678.frc2024.Ports;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.Subsystem;
import com.team1678.lib.requests.Request;
import com.team254.lib.drivers.TalonUtil;
import com.team6647.frc2025.Ports;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorTest extends Subsystem {
	private static MotorTest mInstance;
	private State mState = State.IDLE;
	private PeriodicIO mPeriodicIO = new PeriodicIO();

	private final TalonFX mMotor;

	public enum State {
		IDLE(0.0),
		FORWARD(10.0),
		BACKWARD(-10.0);

		public double feeder_voltage;

		State(double feeder_voltage) {
			this.feeder_voltage = feeder_voltage;
		}
	}

	private MotorTest() {
		mMotor = new TalonFX(Ports.TESTING_MOTOR.getDeviceNumber(), Ports.TESTING_MOTOR.getBus());
		//TalonUtil.applyAndCheckConfiguration(mFeeder, AmpRollerConstants.AmpRollerFXConfig());
		mMotor.setInverted(false);
		mMotor.setNeutralMode(NeutralModeValue.Brake);
	}

	public static MotorTest getInstance() {
		if (mInstance == null) {
			mInstance = new MotorTest();
		}
		return mInstance;
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				mPeriodicIO.demand = mState.feeder_voltage;
			}

			@Override
			public void onStop(double timestamp) {}
		});
	}

	/**
	 * @return The current state of the amp rollers.
	 */
	public State getState() {
		return mState;
	}

	/**
	 * Sets the state of the intake rollers and changes output voltage.
	 *
	 * @param state The state to set.
	 */
	public void setState(State state) {
		mState = state;
	}

	/**
	 * @param _wantedState Wanted state for the amp rollers.
	 * @return New request that updates the amp rollers with the wanted state. 
	 */
	public Request stateRequest(State _wantedState) {
		return new Request() {
			@Override
			public void act() {
				setState(_wantedState);
			}

			@Override
			public boolean isFinished() {
				return mPeriodicIO.demand == _wantedState.feeder_voltage;
			}
		};
	}

	@Override
	public void readPeriodicInputs() {
		mPeriodicIO.output_voltage = mMotor.getMotorVoltage().getValueAsDouble();
		mPeriodicIO.stator_current = mMotor.getStatorCurrent().getValueAsDouble();
		mPeriodicIO.velocity = mMotor.getVelocity().getValueAsDouble();
	}

	@Override
	public void writePeriodicOutputs() {
		mMotor.setControl(new VoltageOut(mPeriodicIO.demand));
	}

	@Override
	public void stop() {
		mPeriodicIO.demand = 0.0;
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	private static class PeriodicIO implements Sendable {
		// Inputs
		private double output_voltage;
		private double stator_current;
		private double velocity;

		// Outputs
		private double demand;

		@Override
		public void initSendable(SendableBuilder builder) {
			builder.addDoubleProperty("Demand", () -> demand, null);
			builder.addDoubleProperty("VelocityRpS", () -> velocity, null);
			builder.addDoubleProperty("OutputVoltage", () -> output_voltage, null);
			builder.addDoubleProperty("StatorCurrent", () -> stator_current, null);
		}
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putString("AmpRollers/Roller State", mState.toString());
		SmartDashboard.putData("AmpRollers/IO", mPeriodicIO);
	}
}