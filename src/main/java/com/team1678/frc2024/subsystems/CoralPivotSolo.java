package com.team1678.frc2024.subsystems;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystemWithCancoder;
import com.team1678.lib.TunableNumber;
import com.team1678.lib.requests.Request;
import com.team1678.lib.util.Stopwatch;
import com.team254.lib.util.Util;
import com.team6647.frc2025.Constants;
import com.team6647.frc2025.Constants.CoralPivotConstants;
import com.team6647.frc2025.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralPivotSolo extends ServoMotorSubsystem {
	private static CoralPivotSolo mInstance;
	private boolean mHoming = false;
	private Stopwatch mHomingStart = new Stopwatch();

	public static CoralPivotSolo getInstance() {
		if (mInstance == null) {
			mInstance = new CoralPivotSolo(
					CoralPivotConstants.kHoodServoConstants);
		}
		return mInstance;
	}

	private static TunableNumber depositing = new TunableNumber(Constants.CoralPivotConstants.kHoodServoConstants.kName + "/POSdepositing", 1, false);
	private static TunableNumber intaking = new TunableNumber(Constants.CoralPivotConstants.kHoodServoConstants.kName + "/POSdeploying", 2, false);

	public enum PivotPosition {
		DEPOSITING(depositing.get()),
		INTAKING(intaking.get());

		private final double position;

		private PivotPosition(double position) {
			this.position = position;
		}

		public double getPosition() {
			return position;
		}
	}

	private CoralPivotSolo(final ServoMotorSubsystemConstants constants) {
		super(constants);
		zeroSensors();
		changeTalonConfig((conf) -> {
			conf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
			conf.Feedback.FeedbackRemoteSensorID = Ports.CORAL_CANCODER.getDeviceNumber();
			conf.Feedback.RotorToSensorRatio = (CoralPivotConstants.kRotorRotationsPerOutputRotation)
					/ (mConstants.kRotationsPerUnitDistance * 360.0);
			conf.Feedback.SensorToMechanismRatio = 1.0;
			return conf;
		});
		setPosition(CoralPivotConstants.kHoodEncoderConstants.remote_encoder_offset);
	}

	@Override
	public void registerEnabledLoops(ILooper mEnabledLooper) {
		mEnabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				//setSetpointMotionMagic(15.0); // Goes to bottom on start
				setSetpointMotionMagic(0.0); // Goes to bottom on start
			}

			@Override
			public void onLoop(double timestamp) {}

			@Override
			public void onStop(double timestamp) {
				setNeutralMode(NeutralModeValue.Brake);
			}
		});
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		if (mHoming) {
			if (Math.abs(mPeriodicIO.main_stator_current) > CoralPivotConstants.kHomingCurrentThreshold
					&& mHomingStart.getTime()
							> CoralPivotConstants.kMinHomingTime) { // Stop homing if we've hit a hardstop
				//setOpenLoop(0.0);
				enableSoftLimits(true);
				mHoming = false;
			} else if (mHomingStart.getTime() > CoralPivotConstants.kMaxHomingTime) {
				mHoming = false;
			} else {
				setOpenLoop(CoralPivotConstants.kHomingVoltage / 12.0);
			}
		}
		super.writePeriodicOutputs();
	}

	@Override
	public synchronized void outputTelemetry() {
		SmartDashboard.putBoolean(mConstants.kName + "/homing", mHoming);
		super.outputTelemetry();
	}
	
	/**
	 * Moves hood relative to current position.
	 * @param delta Delta from current position to travel, in degrees.
	 */
	public synchronized void setWantJog(double delta) {
		setSetpointMotionMagic(getSetpointHomed() + delta);
	}

	/**
	 * Runs hood rehoming sequence.
	 * @param home Enable/Disable rehome sequence.
	 */
	public synchronized void setWantHome(boolean home) {
		if (home && !mHoming) {
			mHoming = true;
			mHomingStart.reset();
			mHomingStart.start();
			enableSoftLimits(false);
		}
	}

	public boolean isHoming() {
		return mHoming;
	}
}