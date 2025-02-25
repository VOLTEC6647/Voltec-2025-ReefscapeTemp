package com.team1678.frc2024.auto.actions;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.team1678.frc2024.subsystems.Drive;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory254;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team6647.frc2025.Robot;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;

public class ChoreoTrajectoryAction implements Action {

	private Drive mDrive = null;

	Optional<Trajectory<SwerveSample>> trajectory;
	private final boolean mResetGyro;

	private Timer autoTimer = new Timer();
				

	public ChoreoTrajectoryAction(String trajectory) {
		this(trajectory, false);
	}

	public ChoreoTrajectoryAction(String trajectoryName, boolean resetPose) {
		trajectory = Choreo.loadTrajectory(trajectoryName);
		mDrive = Drive.getInstance();
		mResetGyro = resetPose;
	}

	@Override
	public void start() {
		autoTimer.reset();
		if (mResetGyro) {
			mDrive.resetOdometry(trajectory.get().getInitialPose(Robot.is_red_alliance).get());
			System.out.println("Reset gyro to " + mDrive.getPose().getRotation().getDegrees());
		}
		
	}

	@Override
	public void update() {
		Optional<SwerveSample> sample = trajectory.get().sampleAt(autoTimer.get(), Robot.is_red_alliance);
		if (sample.isPresent()) {
			mDrive.choreoController(sample.get());
		}else{
			System.out.println("Sample not present");
		}
		System.out.println("Trajectory set");
	}

	@Override
	public boolean isFinished() {
		return trajectory.get().getTotalTime() < autoTimer.get();
	}

	@Override
	public void done() {}
}
