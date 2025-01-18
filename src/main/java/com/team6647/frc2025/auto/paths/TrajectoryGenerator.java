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
import com.team6647.frc2025.controlboard.DriverControls;

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
		public final Trajectory254<TimedState<Pose2dWithMotion>> testT;
		public Trajectory254<TimedState<Pose2dWithMotion>> putCoral;
		public Trajectory254<TimedState<Pose2dWithMotion>> enterCoral;

		private boolean wants_mirrored;

		private TrajectorySet(boolean mirror) {
			wants_mirrored = mirror;
			// spotless:off
			center6 = center6();
			testT = getTestTrajectory();
			putCoral = getPutCoral();
			enterCoral = getEnterCoral();
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
			return generate(waypoints, headings, List.of(), false, 0.5, 1.0);
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
			waypoints.add(new Pose2d(10,20,new Rotation2d()));
			headings.add(Rotation2d.fromDegrees(0.0));
			waypoints.add(new Pose2d(FieldLayout.kCoralCenter.getTranslation().x()-FieldLayout.kCoralDistance,FieldLayout.kCoralCenter.getTranslation().x(),new Rotation2d()));
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 0.4, 1.0);
		}

		public Trajectory254<TimedState<Pose2dWithMotion>> getPutCoral() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			waypoints.add(Drive.getInstance().getPose());
			headings.add(Rotation2d.fromDegrees(0.0));
			if(DriverControls.subCoralId == 1){
				waypoints.add(FieldLayout.getCoralTargetPos(DriverControls.angles[DriverControls.coralId]).coral1Pre);
			}
			if(DriverControls.subCoralId == 2){
				waypoints.add(FieldLayout.getCoralTargetPos(DriverControls.angles[DriverControls.coralId]).coral2Pre);
			}
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.1, 1.0);
		}

		public Trajectory254<TimedState<Pose2dWithMotion>> getEnterCoral() {
			List<Pose2d> waypoints = new ArrayList<>();
			List<Rotation2d> headings = new ArrayList<>();
			if(DriverControls.subCoralId == 1){
				waypoints.add(FieldLayout.getCoralTargetPos(DriverControls.angles[DriverControls.coralId]).coral1Pre);
				headings.add(Rotation2d.fromDegrees(0.0));
				waypoints.add(FieldLayout.getCoralTargetPos(DriverControls.angles[DriverControls.coralId]).coral1);
			}
			if(DriverControls.subCoralId == 2){
				waypoints.add(FieldLayout.getCoralTargetPos(DriverControls.angles[DriverControls.coralId]).coral2Pre);
				headings.add(Rotation2d.fromDegrees(0.0));
				waypoints.add(FieldLayout.getCoralTargetPos(DriverControls.angles[DriverControls.coralId]).coral2);
			}
			headings.add(Rotation2d.fromDegrees(0.0));
			return generate(waypoints, headings, List.of(), false, 1.1, 1.0);
		}
	}
}
