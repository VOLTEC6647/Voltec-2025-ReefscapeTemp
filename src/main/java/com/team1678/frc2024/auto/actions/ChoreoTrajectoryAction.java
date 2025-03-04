package com.team1678.frc2024.auto.actions;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.team1678.frc2024.subsystems.Drive;
import com.team1678.lib.swerve.ChassisSpeeds;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory254;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team6647.Robot;

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
		mDrive.acceptingHeading = 0;
		autoTimer.reset();
		if (mResetGyro) {
			mDrive.resetOdometry(trajectory.get().getInitialPose(Robot.is_red_alliance).get());
			mDrive.zeroGyro(trajectory.get().getInitialPose(Robot.is_red_alliance).get().getRotation().getDegrees());
			System.out.println("Reset odometry to " + mDrive.getPose().getRotation().getDegrees());
		}
		autoTimer.start();		
	}

	@Override
	public void update() {
		Optional<SwerveSample> sample = trajectory.get().sampleAt(autoTimer.get(), Robot.is_red_alliance);
		if (sample.isPresent()) {
			mDrive.choreoController(sample.get());
			//Logger.recordOutput("serrrr",sample.get()); crash
		}else{
			System.out.println("Sample not present");
		}
		if(trajectory.get().getTotalTime() < autoTimer.get()){
			mDrive.acceptingHeading = 0;
		}
		System.out.println("Trajectory set");
	}

	@Override
	public boolean isFinished() {
		return trajectory.get().getTotalTime()+3 < autoTimer.get();
	}

	@Override
	public void done() {
		mDrive.feedTeleopSetpoint(new ChassisSpeeds());
	}
}
