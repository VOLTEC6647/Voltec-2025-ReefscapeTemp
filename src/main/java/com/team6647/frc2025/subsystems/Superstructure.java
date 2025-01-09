package com.team6647.frc2025.subsystems;

import com.team1678.frc2024.Constants1678;
import com.team1678.frc2024.FieldLayout;
import com.team1678.frc2024.Ports1678;
import com.team1678.frc2024.Robot1678;
import com.team1678.frc2024.RobotState;
import com.team1678.frc2024.FieldLayout.CoralTarget;
import com.team1678.frc2024.controlboard.ControlBoard;
import com.team1678.frc2024.led.TimedLEDState;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.shooting.FerryUtil;
import com.team1678.frc2024.shooting.ShootingUtil;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.PeriodicLogs;
import com.team1678.frc2024.subsystems.Subsystem;
import com.team1678.lib.TunableNumber;
import com.team1678.lib.drivers.BeamBreak;
import com.team1678.lib.requests.IfRequest;
import com.team1678.lib.requests.LambdaRequest;
import com.team1678.lib.requests.ParallelRequest;
import com.team1678.lib.requests.Request;
import com.team1678.lib.requests.SequentialRequest;
import com.team1678.lib.requests.WaitRequest;
import com.team254.lib.geometry.Pose2d254;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team6647.frc2025.auto.modes.configuredQuals.test1;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class Superstructure extends Subsystem {

	private static Superstructure mInstance;

	public static synchronized Superstructure getInstance() {
		if (mInstance == null) {
			mInstance = new Superstructure();
		}

		return mInstance;
	}

	// Request tracking variables
	private Request activeRequest = null;
	private ArrayList<Request> queuedRequests = new ArrayList<>(0);
	private boolean hasNewRequest = false;
	private boolean allRequestsComplete = false;

	// Subsystems
	

	// LEDs
	//private final LEDs mLEDs = LEDs.getInstance();
	private TimedLEDState mHeldState = TimedLEDState.NOTE_HELD_SHOT;

	// Target tracking
	private Drive mDrive = Drive.getInstance();
	private double mDistanceToTarget = 0.0;
	private double mAngularErrToTarget = 0.0;

	// Manual param tuning
	public final boolean kUseSmartdash = false;
	public TunableNumber kCurveTuner = new TunableNumber("FiringParams/ManualCurveTune", 0.0, true);
	public TunableNumber kSkewTuner = new TunableNumber("FiringParams/ManualSkewTune", 0.0, true);
	public TunableNumber mHoodTuner = new TunableNumber("FiringParams/ManualHoodTune", 0.0, true);
	public TunableNumber mRPMTuner = new TunableNumber("FiringParams/ManualRPMTune", 0.0, true);

	// Trackers
	private boolean CLIMB_MODE = false;
	private boolean PREP = false;
	private boolean FERRY_SHOT = false;
	private boolean WANTS_SPINDOWN = false;

	public boolean requestsCompleted() {
		return allRequestsComplete;
	}

	public void request(Request r) {
		setActiveRequest(r);
		clearRequestQueue();
	}

	private void setActiveRequest(Request request) {
		activeRequest = request;
		hasNewRequest = true;
		allRequestsComplete = false;
	}

	private void clearRequestQueue() {
		queuedRequests.clear();
	}

	private void setRequestQueue(List<Request> requests) {
		clearRequestQueue();
		for (Request req : requests) {
			queuedRequests.add(req);
		}
	}

	private void setRequestQueue(Request activeRequest, ArrayList<Request> requests) {
		request(activeRequest);
		setRequestQueue(requests);
	}

	private void addRequestToQueue(Request req) {
		queuedRequests.add(req);
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				clearRequestQueue();
			}

			@Override
			public void onLoop(double timestamp) {
				try {
					if (hasNewRequest && activeRequest != null) {
						activeRequest.act();
						hasNewRequest = false;
					}

					if (activeRequest == null) {
						if (queuedRequests.isEmpty()) {
							allRequestsComplete = true;
						} else {
							request(queuedRequests.remove(0));
						}
					} else if (activeRequest.isFinished()) {
						activeRequest = null;
					}

					//updateShootingSetpoints();
				} catch (Exception e) {
					e.printStackTrace();
				}
				PeriodicLogs.periodic();
			}

			@Override
			public void onStop(double timestamp) {
				stop();
			}
		});
	}

	@Override
	public void stop() {
		activeRequest = null;
		clearRequestQueue();
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public void readPeriodicInputs() {

	}

	@Override
	public void outputTelemetry() {

		SmartDashboard.putBoolean("Ferry Shot", FERRY_SHOT);

		
		SmartDashboard.putNumber("FiringParams/Angular Err To Target", mAngularErrToTarget);
	}

	/* Superstructure functions */
	public void go6(boolean ferryShot) {
		new test1();
	}

	// spotless:off 
	
	




	/**
	 * BeamBreak Sensor reading.
	 * 
	 * @param mBreak BeamBreak Sensor.
	 * @param target_state If wanted reading is true (broken) or false (not broken).
	 * 
	 * @return Boolean for if target state is acheived. 
	 */
	private Request breakWait(BeamBreak mBreak, boolean target_state) {
		return new Request() {

			@Override
			public void act() {}

			@Override
			public boolean isFinished() {
				return mBreak.get() == target_state;
			}
		};
	}


	private Request boolWait(BooleanSupplier func) {
		return new Request() {

			@Override
			public void act() {}

			@Override
			public boolean isFinished() {
				return func.getAsBoolean();
			}
		};
	}

	/**
	 * Debounced BeamBreak Sensor reading.
	 * 
	 * @param mBreak BeamBreak Sensor.
	 * @param target_state If wanted reading is true (broken) or false (not broken).
	 * @param delayed_wait_seconds Debounces time from a BeamBreak Sensor. 
	 * 
	 * @return Boolean for if target state is acheived after debouncing the signal. 
	 */
	private Request breakWait(BeamBreak mBreak, boolean target_state, double delayed_wait_seconds) {
		return new Request() {

			TimeDelayedBoolean timeout = new TimeDelayedBoolean();

			@Override
			public void act() {}

			@Override
			public boolean isFinished() {
				return timeout.update(mBreak.get() == target_state, delayed_wait_seconds);
			}
		};
	}


	/**
	 * Stop Intake Rollers, Serializer, Feeder, and Amp Rollers.
	 */
	private Request idleRequest() {
		return new ParallelRequest(

		);
	}

	/**
	 * Stop Intake Rollers, Serializer, Feeder, and Amp Rollers.
	 */
	public void idleState() {
		request(idleRequest());
	}

	ControlBoard mControlBoard = ControlBoard.getInstance();

	public void preGen(){
		
	}
	
	
	// spotless:on
}
