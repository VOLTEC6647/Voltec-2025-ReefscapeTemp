package com.team1678.frc2024.subsystems.limelight;

import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.Map;
import java.util.TreeMap;

/**
 * A class that is used to keep track of all goals detected by the vision system. As goals are detected/not detected
 * anymore by the vision system, function calls will be made to create, destroy, or update a goal track.
 * <p>
 * This helps in the goal ranking process that determines which goal to fire into, and helps to smooth measurements of
 * the goal's location over time.
 *
 * @see GoalTracker
 */
public class GoalTrack {
	final GoalTracker.Configuration mConfiguration;
	TreeMap<Double, Translation2d> mObservedPositions = new TreeMap<>();
	Translation2d mSmoothedPosition = null;
	int mId;

	private GoalTrack(GoalTracker.Configuration config) {
		mConfiguration = config;
	}

	public synchronized void emptyUpdate() {
		pruneByTime();
	}

	/**
	 * Attempts to update the track with a new observation.
	 *
	 * @return True if the track was updated
	 */
	public synchronized boolean tryUpdate(double timestamp, Translation2d new_observation) {
		if (!isAlive()) {
			return false;
		}
		double distance = mSmoothedPosition
				.inverse()
				.translateBy(new_observation)
				.norm(); // Distance between new observation and average position of Goal Tracker
		if (distance < mConfiguration.kMaxTrackerDistance) {
			mObservedPositions.put(timestamp, new_observation);
			pruneByTime();
			return true;
		} else {
			emptyUpdate();
			return false;
		}
	}

	public synchronized boolean isAlive() {
		return mObservedPositions.size() > 0;
	}

	/**
	 * Removes the track if it is older than the set "age" described in the Constants file.
	 *
	 * @see Constants1678
	 */
	synchronized void pruneByTime() {
		double delete_before = Timer.getFPGATimestamp() - mConfiguration.kMaxGoalTrackAge;
		mObservedPositions.entrySet().removeIf(entry -> entry.getKey() < delete_before);
		if (mObservedPositions.isEmpty()) {
			mSmoothedPosition = null;
		} else {
			smooth();
		}
	}

	/**
	 * Makes a new track based on the timestamp and the goal's coordinates (from vision)
	 */
	public static GoalTrack makeNewTrack(
			GoalTracker.Configuration config, double timestamp, Translation2d first_observation, int id) {
		GoalTrack rv = new GoalTrack(config);
		rv.mObservedPositions.put(timestamp, first_observation);
		rv.mSmoothedPosition = first_observation;
		rv.mId = id;
		return rv;
	}

	/**
	 * Averages out the observed positions based on an set of observed positions
	 */
	synchronized void smooth() {
		if (isAlive()) {
			double x = 0;
			double y = 0;
			double t_now = Timer.getFPGATimestamp();
			int num_samples = 0;
			for (Map.Entry<Double, Translation2d> entry : mObservedPositions.entrySet()) {
				if (t_now - entry.getKey() > mConfiguration.kMaxGoalTrackSmoothingTime) {
					continue;
				}
				++num_samples;
				x += entry.getValue().getTranslation().x();
				y += entry.getValue().getTranslation().y();
			}
			x /= num_samples;
			y /= num_samples;

			if (num_samples == 0) {
				// Handle the case that all samples are older than kMaxGoalTrackSmoothingTime.
				mSmoothedPosition = mObservedPositions.lastEntry().getValue();
			} else {
				mSmoothedPosition = new Translation2d(x, y);
			}
		}
	}

	public synchronized Translation2d getSmoothedPosition() {
		return mSmoothedPosition;
	}

	public synchronized Translation2d getLatestPosition() {
		return mObservedPositions.lastEntry().getValue();
	}

	public synchronized double getLatestTimestamp() {
		return mObservedPositions.keySet().stream().max(Double::compareTo).orElse(0.0);
	}

	public synchronized double getStability() {
		return Math.min(
				1.0, mObservedPositions.size() / (mConfiguration.kCameraFrameRate * mConfiguration.kMaxGoalTrackAge));
	}

	public synchronized int getId() {
		return mId;
	}
}
