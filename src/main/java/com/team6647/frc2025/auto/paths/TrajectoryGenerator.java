package com.team6647.frc2025.auto.paths;

import com.team1678.frc2024.Constants1678;
import com.team1678.frc2024.FieldLayout;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.lib.swerve.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory254;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;

public class TrajectoryGenerator {

	private static TrajectoryGenerator mInstance;

	public static TrajectoryGenerator getInstance() {
		if (mInstance == null) {
			mInstance = new TrajectoryGenerator(Drive.getInstance().getMotionPlanner());
		}
		return mInstance;
	}

	private static final double kMaxAccel = 6.0;
	private static final double kMaxVoltage = 9.0;

	private final DriveMotionPlanner mMotionPlanner;
	private TrajectorySet mTrajectorySet = null;

	public TrajectoryGenerator(DriveMotionPlanner motion_planner) {
		mMotionPlanner = motion_planner;
	}

	public void generateTrajectories() {
		if (mTrajectorySet == null) {
			System.out.println("Generating trajectories...");
			mTrajectorySet = new TrajectorySet();
			System.out.println("Finished trajectory generation");
		}
	}

	public void forceRegenerateTrajectories(boolean mirror) {
		System.out.println("Generating trajectories..." + (mirror ? " MIRRORED VERSION" : ""));
		mTrajectorySet = new TrajectorySet(mirror);
		System.out.println("Finished trajectory generation");
	}

	public TrajectorySet getTrajectorySet() {
		return mTrajectorySet;
	}

	public Trajectory254<TimedState<Pose2dWithMotion>> generateTrajectory(
			boolean reversed,
			final List<Pose2d> waypoints,
			final List<Rotation2d> headings,
			final List<TimingConstraint<Pose2dWithMotion>> constraints,
			double max_vel, // m/s
			double max_accel, // m/s^2
			double max_voltage) {
		return mMotionPlanner.generateTrajectory(
				reversed, waypoints, headings, constraints, max_vel, max_accel, max_voltage);
	}

	public Trajectory254<TimedState<Pose2dWithMotion>> generateTrajectory(
			boolean reversed,
			final List<Pose2d> waypoints,
			final List<Rotation2d> headings,
			final List<TimingConstraint<Pose2dWithMotion>> constraints,
			double start_vel, // m/s
			double end_vel, // m/s
			double max_vel, // m/s
			double max_accel, // m/s^2
			double max_voltage) {
		return mMotionPlanner.generateTrajectory(
				reversed, waypoints, headings, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
	}

	public class TrajectorySet {

		public final Trajectory254<TimedState<Pose2dWithMotion>> center6;

		private boolean wants_mirrored;

		private TrajectorySet(boolean mirror) {
			wants_mirrored = mirror;
			// spotless:off
			center6 = center6();
			// spotless:on
		}

		private TrajectorySet() {
			this(false);
		}

		private void convertToM(List<Pose2d> waypoints, List<Rotation2d> headings) {
			for (int i = 0; i < waypoints.size(); ++i) {
				System.out.println("waypoints.add(new Pose2d("
						+ Units.inchesToMeters(waypoints.get(i).getTranslation().x())
						+ ", "
						+ Units.inchesToMeters(waypoints.get(i).getTranslation().y())
						+ ", Rotation2d.fromDegrees("
						+ waypoints.get(i).getRotation().getDegrees()
						+ ")));");
				System.out.println(
						"headings.add(Rotation2d.fromDegrees(" + headings.get(i).getDegrees() + "));");
			}
			System.out.println("\n\n");
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> generate(
				List<Pose2d> waypoints,
				List<Rotation2d> headings,
				List<TimingConstraint<Pose2dWithMotion>> constraints,
				boolean reversed,
				double percentSpeed,
				double percentAccel) {

			handleAllianceFlip(waypoints, headings);
			return generateTrajectory(
					reversed,
					waypoints,
					headings,
					constraints,
					percentSpeed * Constants1678.SwerveConstants.maxAutoSpeed,
					percentAccel * kMaxAccel,
					kMaxVoltage);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> generate(
				List<Pose2d> waypoints,
				List<Rotation2d> headings,
				List<TimingConstraint<Pose2dWithMotion>> constraints,
				boolean reversed,
				double percentSpeed,
				double percentAccel,
				double start_vel,
				double end_vel) {
			handleAllianceFlip(waypoints, headings);
			Trajectory254<TimedState<Pose2dWithMotion>> trajectory = generateTrajectory(
					reversed,
					waypoints,
					headings,
					constraints,
					start_vel * Constants1678.SwerveConstants.maxAutoSpeed,
					end_vel * Constants1678.SwerveConstants.maxAutoSpeed,
					percentSpeed * Constants1678.SwerveConstants.maxAutoSpeed,
					percentAccel * kMaxAccel,
					kMaxVoltage);
			return trajectory;
		}

		private void handleAllianceFlip(List<Pose2d> waypoints, List<Rotation2d> headings) {
			if (wants_mirrored) {
				for (int i = 0; i < waypoints.size(); i++) {
					waypoints.set(
							i,
							new Pose2d(
									waypoints.get(i).getTranslation().mirrorAboutX(FieldLayout.kFieldLength / 2.0),
									waypoints.get(i).getRotation().mirrorAboutX()));
				}
				for (int i = 0; i < headings.size(); i++) {
					headings.set(i, headings.get(i).mirrorAboutX());
				}
			}
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getTestTrajectory() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.5, 1.5, Rotation2d.fromDegrees(0)));
			headings.add(Rotation2d.fromDegrees(180.0));
			waypoints.add(new Pose2d(7.0, 1.5, Rotation2d.fromDegrees(0)));
			headings.add(Rotation2d.fromDegrees(180.0));
			return generate(waypoints, headings, List.of(), false, 0.8, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getTestTrajectory2() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(7.0, 1.5, Rotation2d.fromDegrees(180)));
			headings.add(Rotation2d.fromDegrees(180.0));
			waypoints.add(new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(180)));
			headings.add(Rotation2d.fromDegrees(180.0));
			return generate(waypoints, headings, List.of(), false, 0.8, 1.0);
		}

		// L6 PATHS
		private Trajectory254<TimedState<Pose2dWithMotion>> getL6RightStartToRightPickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(1.4, 5.592, Rotation2d.fromDegrees(-20.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(2.6, 4.3, Rotation2d.fromDegrees(-20.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 0.5, 0.4);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getL6RightPickupToMidShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.6, 4.3, Rotation2d.fromDegrees(150.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(2.0, 5.5, Rotation2d.fromDegrees(60.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(2.6, 5.6, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.5);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getL6MidPickupToLeftShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.6, 5.6, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(2.6, 6.8, Rotation2d.fromDegrees(10.0)));
			headings.add(Rotation2d.fromDegrees(20.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.5, 0.0, 0.1);
		}

		// Primary L6
		private Trajectory254<TimedState<Pose2dWithMotion>> getL6LeftShotToCenterPickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.6, 6.8, Rotation2d.fromDegrees(10.0)));
			headings.add(Rotation2d.fromDegrees(20.0));
			waypoints.add(new Pose2d(6.0, 7.25, Rotation2d.fromDegrees(8.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.5, 7.45, Rotation2d.fromDegrees(-0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.3, 0.1, 0.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getL6CenterPickupToLeftShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.5, 7.45, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(4.17, 6.0, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.3);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getL6LeftShotToFarLeftCenterPickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(4.17, 6.0, Rotation2d.fromDegrees(10.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(5.8, 6.6, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.4, 5.75, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getL6FarLeftCenterPickupToLeftShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 5.75, Rotation2d.fromDegrees(160.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(4.17, 6.0, Rotation2d.fromDegrees(200.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.3);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getL6LeftShotToN3() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(4.17, 6.0, Rotation2d.fromDegrees(-80.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.4, 4.1, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getL6N3ToLeftShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 4.1, Rotation2d.fromDegrees(120.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(4.5, 6.7, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.3);
		}

		// Alternate L6
		private Trajectory254<TimedState<Pose2dWithMotion>> getAL6LeftShotToCenterPickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.6, 6.8, Rotation2d.fromDegrees(20.0)));
			headings.add(Rotation2d.fromDegrees(55.0));
			waypoints.add(new Pose2d(6.0, 7.0, Rotation2d.fromDegrees(-10.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.4, 5.75, Rotation2d.fromDegrees(-30.0)));
			headings.add(Rotation2d.fromDegrees(-30.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.7, 0.1, 0.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getAL6CenterPickupToLeftShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 5.75, Rotation2d.fromDegrees(160.0)));
			headings.add(Rotation2d.fromDegrees(-30.0));
			waypoints.add(new Pose2d(4.17, 6.5, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getAL6LeftShotToSecondCenterPickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(4.17, 6.5, Rotation2d.fromDegrees(0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.3, 4.1, Rotation2d.fromDegrees(-50.0)));
			headings.add(Rotation2d.fromDegrees(-50.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getAL6SecondCenterPickupToLeftShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.3, 4.1, Rotation2d.fromDegrees(130.0)));
			headings.add(Rotation2d.fromDegrees(-50.0));
			waypoints.add(new Pose2d(4.17, 6.5, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		// Semi Fast Six
		private Trajectory254<TimedState<Pose2dWithMotion>> getSF6StartToS1Shot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(1.5, 6.6, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(20.0));
			waypoints.add(new Pose2d(2.7, 6.8, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(20.0));
			return generate(waypoints, headings, List.of(), false, 0.07, 1.0, 0.0, 0.12);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getSF6S1ShotToN1Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.7, 6.8, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(20.0));
			waypoints.add(new Pose2d(4.0, 6.8, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.5, 7.45, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.7, 0.12, 0.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getSF6N1PickupToFarShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.5, 7.45, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(4.8, 6.5, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getSF6FarShotToN2Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(4.8, 6.5, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(15.0));
			waypoints.add(new Pose2d(5.5, 6.5, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(-30.0));
			waypoints.add(new Pose2d(8.4, 5.75, Rotation2d.fromDegrees(-30.0)));
			headings.add(Rotation2d.fromDegrees(-35.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.8);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getSF6N2PickupToNearShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 5.75, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(-35.0));
			waypoints.add(new Pose2d(4.8, 6.5, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(1.8, 6.5, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getSF6NearShotToS2Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(1.8, 6.5, Rotation2d.fromDegrees(-90.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(2.2, 5.5, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(2.7, 5.5, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getSF6S2PickupToS3Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.7, 5.5, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(2.1, 4.6, Rotation2d.fromDegrees(-90.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(2.7, 4.45, Rotation2d.fromDegrees(-20.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 0.7, 0.5);
		}

		// SF6 Backup Paths
		private Trajectory254<TimedState<Pose2dWithMotion>> getSF6N1ToN2() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.5, 7.45, Rotation2d.fromDegrees(-160.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(7.8, 6.6, Rotation2d.fromDegrees(-90.0)));
			headings.add(Rotation2d.fromDegrees(-45.0));
			waypoints.add(new Pose2d(8.3, 5.9, Rotation2d.fromDegrees(-50.0)));
			headings.add(Rotation2d.fromDegrees(-40.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.6);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getSF6N2PickupToFarShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.3, 5.8, Rotation2d.fromDegrees(150.0)));
			headings.add(Rotation2d.fromDegrees(-40.0));
			waypoints.add(new Pose2d(4.8, 6.5, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.5);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getSF6FarShotToN3Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(4.5, 6.5, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(7.0, 5.6, Rotation2d.fromDegrees(-50.0)));
			headings.add(Rotation2d.fromDegrees(-70.0));
			waypoints.add(new Pose2d(8.3, 4.0, Rotation2d.fromDegrees(-70.0)));
			headings.add(Rotation2d.fromDegrees(-65.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getSF6N3PickupToNearShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.3, 4.0, Rotation2d.fromDegrees(120.0)));
			headings.add(Rotation2d.fromDegrees(-65.0));
			waypoints.add(new Pose2d(4.5, 6.5, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(1.8, 6.5, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.5);
		}

		// Adaptive Six
		private Trajectory254<TimedState<Pose2dWithMotion>> getA6StartToS2() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(1.4, 5.592, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(2.7, 5.592, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 0.5, 0.3, 0.0, 0.5);
		}

		// N1 Branch
		private Trajectory254<TimedState<Pose2dWithMotion>> getA6S2ToN1() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.7, 5.592, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(7.0, 7.45, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.5, 7.45, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.4, 1.0, 0.5, 0.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6N1ToFarShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.5, 7.45, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(4.8, 6.5, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6FarShotToN1() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(4.8, 6.5, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.5, 7.45, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6N1ToNearShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.5, 7.45, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(1.8, 5.592, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		// N2 Branch
		private Trajectory254<TimedState<Pose2dWithMotion>> getA6S2ToN2() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.7, 5.592, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(5.7, 6.5, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.4, 5.75, Rotation2d.fromDegrees(-30.0)));
			headings.add(Rotation2d.fromDegrees(-30.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0, 0.5, 0.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6N2ToFarShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 5.75, Rotation2d.fromDegrees(150.0)));
			headings.add(Rotation2d.fromDegrees(-35.0));
			waypoints.add(new Pose2d(4.8, 6.5, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6N2ToNearShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 5.75, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(-35.0));
			waypoints.add(new Pose2d(4.8, 6.5, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(1.8, 5.592, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6FarShotToN2() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(4.8, 6.5, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.4, 5.75, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		// N3 Branch
		private Trajectory254<TimedState<Pose2dWithMotion>> getA6S2ToN3() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.7, 5.592, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(6.0, 4.1, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.4, 4.1, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0, 0.5, 0.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6N3ToFarShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 4.1, Rotation2d.fromDegrees(120.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(4.8, 6.5, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6FarShotToN3() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(4.8, 6.5, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.3, 4.0, Rotation2d.fromDegrees(-70.0)));
			headings.add(Rotation2d.fromDegrees(-75.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6N3ToNearShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 4.1, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(6.0, 4.1, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(2.7, 5.592, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(1.8, 5.592, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		// Spike Paths
		private Trajectory254<TimedState<Pose2dWithMotion>> getA6NearShotToS1() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(1.8, 5.592, Rotation2d.fromDegrees(90.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(2.7, 6.8, Rotation2d.fromDegrees(10.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.5);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6S1ToS3() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.7, 6.8, Rotation2d.fromDegrees(190.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(1.8, 5.592, Rotation2d.fromDegrees(-90.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(2.7, 4.45, Rotation2d.fromDegrees(-20.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 0.7, 0.5);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6S3ToDriveout() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.7, 4.45, Rotation2d.fromDegrees(90.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(7.0, 7.0, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6NearShotToS3() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(1.8, 5.592, Rotation2d.fromDegrees(-90.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(2.7, 4.4, Rotation2d.fromDegrees(-20.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.5);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6S3ToS1() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.7, 4.4, Rotation2d.fromDegrees(160.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(1.8, 5.592, Rotation2d.fromDegrees(90.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(2.7, 6.8, Rotation2d.fromDegrees(10.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 0.7, 0.5);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6S1ToDriveout() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.7, 6.8, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(7.0, 7.0, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 1.0);
		}

		// Backup Paths
		private Trajectory254<TimedState<Pose2dWithMotion>> getA6N1ToN2() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.5, 7.45, Rotation2d.fromDegrees(-160.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(7.8, 6.6, Rotation2d.fromDegrees(-90.0)));
			headings.add(Rotation2d.fromDegrees(-45.0));
			waypoints.add(new Pose2d(8.4, 5.75, Rotation2d.fromDegrees(-50.0)));
			headings.add(Rotation2d.fromDegrees(-40.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.6);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6N2ToN1() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 5.75, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(7.8, 6.6, Rotation2d.fromDegrees(90.0)));
			headings.add(Rotation2d.fromDegrees(20.0));
			waypoints.add(new Pose2d(8.5, 7.45, Rotation2d.fromDegrees(20.0)));
			headings.add(Rotation2d.fromDegrees(20.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.6);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6N2ToN3() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 5.75, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(7.8, 5.0, Rotation2d.fromDegrees(-90.0)));
			headings.add(Rotation2d.fromDegrees(-20.0));
			waypoints.add(new Pose2d(8.4, 4.1, Rotation2d.fromDegrees(-20.0)));
			headings.add(Rotation2d.fromDegrees(-20.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.6);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6N3ToN2() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 4.1, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(7.8, 5.0, Rotation2d.fromDegrees(90.0)));
			headings.add(Rotation2d.fromDegrees(20.0));
			waypoints.add(new Pose2d(8.4, 5.75, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(20.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.6);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6N3ToN1() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 4.1, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.4, 7.45, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.6);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getA6N1ToN3() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 7.45, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.4, 4.1, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.6);
		}

		// R3
		private Trajectory254<TimedState<Pose2dWithMotion>> getR3StartToN5Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(1.4, 2, Rotation2d.fromDegrees(-9.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.4, 0.8, Rotation2d.fromDegrees(-9.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.15, 1.1);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3FarShotToN5Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(4.0, 2.8, Rotation2d.fromDegrees(-60.0)));
			headings.add(Rotation2d.fromDegrees(-45.0));
			waypoints.add(new Pose2d(8.4, 0.8, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.15, 1.1);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3N5PickupToFarShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 0.8, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(4.0, 2.8, Rotation2d.fromDegrees(120.0)));
			headings.add(Rotation2d.fromDegrees(-45.0));
			return generate(waypoints, headings, List.of(), false, 1.15, 1.1);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3N5PickupToNearShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 0.8, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(3.5, 3.0, Rotation2d.fromDegrees(135.0)));
			headings.add(Rotation2d.fromDegrees(-40.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.8);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3StartToN4Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(1.4, 2.0, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(5.0, 1.4, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.4, 2.45, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.15, 1.1);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3FarShotToN4Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(4.0, 2.8, Rotation2d.fromDegrees(-50.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(6.0, 1.6, Rotation2d.fromDegrees(10.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.4, 2.45, Rotation2d.fromDegrees(10.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.15, 1.1);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3StageShotToN4Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(4.1, 5.0, Rotation2d.fromDegrees(-25.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(6.0, 4.0, Rotation2d.fromDegrees(-30.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.4, 2.45, Rotation2d.fromDegrees(-5.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.15, 1.1);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3N4PickupToFarShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 2.45, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(6.0, 1.6, Rotation2d.fromDegrees(195.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(4.0, 2.8, Rotation2d.fromDegrees(130.0)));
			headings.add(Rotation2d.fromDegrees(-45.0));
			return generate(waypoints, headings, List.of(), false, 1.15, 1.1);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3N4PickupToStageShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 2.45, Rotation2d.fromDegrees(150.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(4.1, 5.0, Rotation2d.fromDegrees(150.0)));
			headings.add(Rotation2d.fromDegrees(-45.0));
			return generate(waypoints, headings, List.of(), false, 1.15, 1.1);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3N4PickupToNearShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 2.45, Rotation2d.fromDegrees(185.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(6.0, 1.6, Rotation2d.fromDegrees(185.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(3.5, 3.0, Rotation2d.fromDegrees(132.5)));
			headings.add(Rotation2d.fromDegrees(-40.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.8);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3StartToN3Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(1.4, 2.0, Rotation2d.fromDegrees(22.5)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(7.45, 4.1, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.45, 4.1, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.15, 1.1);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3StageShotToN3Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(4.1, 5.0, Rotation2d.fromDegrees(-30.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.45, 4.1, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.15, 1.1);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3FarShotToN3Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(4.0, 2.8, Rotation2d.fromDegrees(60.0)));
			headings.add(Rotation2d.fromDegrees(-45.0));
			waypoints.add(new Pose2d(8.45, 4.1, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.15, 1.1);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3N3PickupToFarShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.45, 4.1, Rotation2d.fromDegrees(-180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(4.0, 2.8, Rotation2d.fromDegrees(-120.0)));
			headings.add(Rotation2d.fromDegrees(-45.0));
			return generate(waypoints, headings, List.of(), false, 1.15, 1.1);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3N3PickupToStageShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.45, 4.1, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(4.1, 5.0, Rotation2d.fromDegrees(145.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.15, 1.1);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3N3PickupToNearShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.45, 4.1, Rotation2d.fromDegrees(-180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(6.2, 4.1, Rotation2d.fromDegrees(-180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(3.5, 3.0, Rotation2d.fromDegrees(-145.0)));
			headings.add(Rotation2d.fromDegrees(-40.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.8);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3ShotToPreloadPickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(3.5, 3.0, Rotation2d.fromDegrees(-155.0)));
			headings.add(Rotation2d.fromDegrees(-120.0));
			waypoints.add(new Pose2d(2.87, 2.7, Rotation2d.fromDegrees(-155.0)));
			headings.add(Rotation2d.fromDegrees(-160.0));
			waypoints.add(new Pose2d(1.4, 2.0, Rotation2d.fromDegrees(-155.0)));
			headings.add(Rotation2d.fromDegrees(-160.0));
			return generate(waypoints, headings, List.of(), false, 0.8, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3PreloadPickupToShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(1.4, 2.0, Rotation2d.fromDegrees(25.0)));
			headings.add(Rotation2d.fromDegrees(-160.0));
			waypoints.add(new Pose2d(3.5, 3.0, Rotation2d.fromDegrees(25.0)));
			headings.add(Rotation2d.fromDegrees(-40.0));
			return generate(waypoints, headings, List.of(), false, 0.8, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR3PreloadShotToTeleStart() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(3.5, 3.0, Rotation2d.fromDegrees(-45.0)));
			headings.add(Rotation2d.fromDegrees(-40.0));
			waypoints.add(new Pose2d(7.4, 1.0, Rotation2d.fromDegrees(-10.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.1, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR5StartToN5Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(1.4, 2, Rotation2d.fromDegrees(-9.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.4, 0.75, Rotation2d.fromDegrees(-9.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.15, 1.1);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR5N5PickupToNearShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 0.75, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(3.5, 3.0, Rotation2d.fromDegrees(135.0)));
			headings.add(Rotation2d.fromDegrees(-40.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.8);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR5StartToN4Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(1.4, 2.0, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(5.0, 1.4, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(8.4, 2.45, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.1, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR5N4PickupToNearShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(8.4, 2.45, Rotation2d.fromDegrees(185.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(6.0, 1.6, Rotation2d.fromDegrees(185.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(3.5, 3.0, Rotation2d.fromDegrees(132.5)));
			headings.add(Rotation2d.fromDegrees(-40.0));
			return generate(waypoints, headings, List.of(), false, 1.1, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR5NearShotToPreloadPickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(3.5, 3.0, Rotation2d.fromDegrees(-155.0)));
			headings.add(Rotation2d.fromDegrees(-40.0));
			waypoints.add(new Pose2d(2.87, 2.7, Rotation2d.fromDegrees(-155.0)));
			headings.add(Rotation2d.fromDegrees(-160.0));
			waypoints.add(new Pose2d(1.4, 2.0, Rotation2d.fromDegrees(-155.0)));
			headings.add(Rotation2d.fromDegrees(-160.0));
			return generate(waypoints, headings, List.of(), false, 1.1, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR5PreloadPickupToShot() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(1.4, 2.0, Rotation2d.fromDegrees(90.0)));
			headings.add(Rotation2d.fromDegrees(-160.0));
			waypoints.add(new Pose2d(2.0, 4.3, Rotation2d.fromDegrees(50.0)));
			headings.add(Rotation2d.fromDegrees(-40.0));
			return generate(waypoints, headings, List.of(), false, 0.8, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR5PreloadShotToS3Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.0, 4.3, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(-40.0));
			waypoints.add(new Pose2d(2.7, 4.3, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(-40.0));
			return generate(waypoints, headings, List.of(), false, 0.6, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR5S3PickupToS2Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.7, 4.3, Rotation2d.fromDegrees(150.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(2.0, 5.5, Rotation2d.fromDegrees(60.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(2.6, 5.6, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.5);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR5S2PickupToS1Pickup() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.6, 5.6, Rotation2d.fromDegrees(180.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(2.6, 6.8, Rotation2d.fromDegrees(10.0)));
			headings.add(Rotation2d.fromDegrees(20.0));
			return generate(waypoints, headings, List.of(), false, 1.0, 0.5);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> getR5S1PickupToTeleStart() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(2.6, 6.8, Rotation2d.fromDegrees(-110.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(4.6, 0.7, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(7.6, 0.7, Rotation2d.fromDegrees(0.0)));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.1, 1.0);
		}

		private Trajectory254<TimedState<Pose2dWithMotion>> center6() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(new Pose2d(FieldLayout.kCoralCenter.getTranslation().x()-FieldLayout.kCoralDistance,FieldLayout.kCoralCenter.getTranslation().x(),new Rotation2d()));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.1, 1.0);
		}
	}
}
