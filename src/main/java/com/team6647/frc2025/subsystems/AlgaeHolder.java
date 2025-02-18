package com.team6647.frc2025.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.Subsystem;
import com.team1678.lib.requests.Request;
import com.team254.lib.drivers.TalonUtil;
import com.team6647.frc2025.Constants.AlgaeHolderConstants;
import com.team6647.frc2025.Constants.AlgaeRollerConstants;
import com.team6647.frc2025.Ports;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeHolder extends Subsystem {
	private static AlgaeHolder mInstance;
	//private double isMoving = false;
	private double lastChangeTimestamp;
	private double lastPosition;

	public static AlgaeHolder getInstance() {
		if (mInstance == null) {
			mInstance = new AlgaeHolder();
		}
		return mInstance;
	}

	public enum State {
		IDLE(0.0),
		DEPLOYING(1.0),
		RETRACTING(-1.0);

		public double holder_voltage;

		State(double holder_voltage) {
			this.holder_voltage = holder_voltage;
		}
	}

	private final SparkMax mHolder;

	private State mState = State.IDLE;
	private PeriodicIO mPeriodicIO = new PeriodicIO();

	private AlgaeHolder() {
        mHolder = new SparkMax(Ports.ALGAE_HOLDER.getDeviceNumber(), MotorType.kBrushless);
        mHolder.configure(AlgaeHolderConstants.SparkMaxConfig(),ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		mHolder.setInverted(true);
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				lastChangeTimestamp = timestamp;
				lastPosition = mHolder.getEncoder().getPosition();
			}

			@Override
			public void onLoop(double timestamp) {
				mPeriodicIO.holder_demand = mState.holder_voltage;
				if(mState == State.IDLE){
					lastChangeTimestamp = timestamp;
					lastPosition = mHolder.getEncoder().getPosition();
				}else{
					if (timestamp-lastChangeTimestamp>0.5){
						if(Math.abs(Math.abs(mHolder.getEncoder().getPosition())-Math.abs(lastPosition))<AlgaeHolderConstants.movementThreshold){
							mPeriodicIO.moving = false;
							setState(State.IDLE);
						}else{
							mPeriodicIO.moving = true;
							lastChangeTimestamp = timestamp;
							lastPosition = mHolder.getEncoder().getPosition();
						}
					}
				}
			}

			@Override
			public void onStop(double timestamp) {}
		});
	}

	private static class PeriodicIO implements Sendable {
		// Inputs
		private double holder_output_voltage;
		private double holder_stator_current;

		private double positionSegment;
		private boolean moving = false;

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
		mPeriodicIO.holder_output_voltage = mHolder.getAppliedOutput();
		mPeriodicIO.holder_stator_current = mHolder.getOutputCurrent();
	}

	@Override
	public void writePeriodicOutputs() {
		mHolder.setVoltage(mPeriodicIO.holder_demand);
		
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
		SmartDashboard.putNumber("AlgaeHolder/Position", mHolder.getEncoder().getPosition());
		SmartDashboard.putBoolean("AlgaeHolder/Moving", mPeriodicIO.moving);
		SmartDashboard.putNumber("AlgaeHolder/LastTimestamp", lastChangeTimestamp);
	}
}