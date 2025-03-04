package com.team1678.frc2024.auto.modes.three;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.LambdaAction;
import com.team1678.frc2024.auto.actions.ParallelAction;
import com.team1678.frc2024.auto.actions.SeriesAction;
import com.team1678.frc2024.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2024.auto.actions.TurnInPlaceAction;
import com.team1678.frc2024.auto.actions.WaitAction;
import com.team1678.frc2024.auto.actions.WaitForSuperstructureAction;
import com.team1678.frc2024.auto.actions.WaitToPassXCoordinateAction;
import com.team1678.frc2024.paths.TrajectoryGenerator1678;
import com.team1678.frc2024.paths.TrajectoryGenerator1678.TrajectorySet;
import com.team1678.frc2024.subsystems.Drive;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory254;
import com.team254.lib.trajectory.timing.TimedState;
import com.team6647.subsystems.Superstructure;

import java.util.List;

public class ThreeNoteMode34 extends AutoModeBase {
	private Drive d = Drive.getInstance();
	private Superstructure s = Superstructure.getInstance();

	Trajectory254<TimedState<Pose2dWithMotion>> startToN3Pickup;
	Trajectory254<TimedState<Pose2dWithMotion>> N4PickupToNearShot;
	Trajectory254<TimedState<Pose2dWithMotion>> N3PickupToStageShot;
	Trajectory254<TimedState<Pose2dWithMotion>> shotToN4Pickup;
	Trajectory254<TimedState<Pose2dWithMotion>> shotToPreLoadPickup;
	Trajectory254<TimedState<Pose2dWithMotion>> preloadPickupToShot;
	Trajectory254<TimedState<Pose2dWithMotion>> preloadShotToTeleStart;

	public ThreeNoteMode34() {
		TrajectorySet s = TrajectoryGenerator1678.getInstance().getTrajectorySet();
		startToN3Pickup = logTrajectory(s.R3StartToN3Pickup);
		N3PickupToStageShot = logTrajectory(s.R3N3PickupToStageShot);
		shotToN4Pickup = logTrajectory(s.R3StageShotToN4Pickup);
		N4PickupToNearShot = logTrajectory(s.R3N4PickupToNearShot);
		shotToPreLoadPickup = logTrajectory(s.R3ShotToPreLoadPickup);
		preloadPickupToShot = logTrajectory(s.R3PreloadPickupToShot);
		preloadShotToTeleStart = logTrajectory(s.R3PreloadShotToTeleStart);
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(new ParallelAction(List.of(
				new SwerveTrajectoryAction(startToN3Pickup, false),
				new SeriesAction(
						new LambdaAction(() -> d.overrideHeading(false)),
						new WaitAction(0.6)))));


		runAction(new ParallelAction(List.of(
				new SwerveTrajectoryAction(N3PickupToStageShot, false),
				new SeriesAction(
						new WaitToPassXCoordinateAction(6.5),
						new LambdaAction(() -> d.overrideHeading(true))))));

		runAction(new WaitAction(0.2));		

		System.out.println("Finished auto!");
	}
	// spotless:on
}
