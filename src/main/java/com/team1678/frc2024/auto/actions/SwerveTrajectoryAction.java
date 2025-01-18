package com.team1678.frc2024.auto.actions;

import org.littletonrobotics.junction.Logger;

import com.team1678.frc2024.subsystems.Drive;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory254;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;

public class SwerveTrajectoryAction implements Action {

	private Drive mDrive = null;

	private final TrajectoryIterator<TimedState<Pose2dWithMotion>> mTrajectory;
	private final boolean mResetGyro;

	public SwerveTrajectoryAction(Trajectory254<TimedState<Pose2dWithMotion>> trajectory) {
		this(trajectory, false);
	}

	public SwerveTrajectoryAction(Trajectory254<TimedState<Pose2dWithMotion>> trajectory, boolean resetPose) {
		mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
		mDrive = Drive.getInstance();
		mResetGyro = resetPose;
		Logger.recordOutput("/Auto/SwerveTrajectoryInit", true);
	}

	@Override
	public void start() {
		Logger.recordOutput("/Auto/SwerveTrajectoryStart", true);
		if (mResetGyro) {
			double newRotation = mTrajectory.getState().state().getRotation().getDegrees();
			System.out.println("Reset gyro to " + newRotation);
			mDrive.zeroGyro(newRotation);
		}
		mDrive.setTrajectory(mTrajectory);
		System.out.println("Trajectory set");
	}

	@Override
	public void update() {}

	@Override
	public boolean isFinished() {
		return mDrive.isDoneWithTrajectory();
	}

	@Override
	public void done() {}
}
