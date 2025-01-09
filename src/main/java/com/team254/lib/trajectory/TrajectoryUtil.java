package com.team254.lib.trajectory;

import com.team254.lib.geometry.*;
import com.team254.lib.spline.QuinticHermitePoseSplineNonholonomic;
import com.team254.lib.spline.PoseSpline;
import com.team254.lib.spline.QuinticHermitePoseSplineHolonomic;
import com.team254.lib.spline.SplineGenerator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.Util;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryUtil {
    public static <S extends IPose2d<S>> Trajectory254<S> mirror(final Trajectory254<S> trajectory) {
        List<S> waypoints = new ArrayList<>(trajectory.length());
        for (int i = 0; i < trajectory.length(); ++i) {
            waypoints.add(trajectory.getPoint(i).state().mirror());
        }
        return new Trajectory254<>(waypoints);
    }

    public static <S extends IPose2d<S>> Trajectory254<TimedState<S>> mirrorTimed(final Trajectory254<TimedState<S>> trajectory) {    // todo fix
        List<TimedState<S>> waypoints = new ArrayList<>(trajectory.length());
        for (int i = 0; i < trajectory.length(); ++i) {
            TimedState<S> timed_state = trajectory.getPoint(i).state();
            waypoints.add(new TimedState<>(timed_state.state().mirror(), timed_state.t(), timed_state.velocity(), timed_state.acceleration()));
        }
        return new Trajectory254<>(waypoints);
    }

    public static <S extends IPose2d<S>> Trajectory254<TimedState<S>> mirrorAboutXTimed(final Trajectory254<TimedState<S>> trajectory, double xValue, double defaultVelocity) {
        List<TimedState<S>> waypoints = new ArrayList<>(trajectory.length());
        for (int i = 0; i < trajectory.length(); ++i) {
            TimedState<S> timed_state = trajectory.getPoint(i).state();
            waypoints.add(new TimedState<S>(timed_state.state().mirrorAboutX(xValue), timed_state.t(), timed_state.velocity(), timed_state.acceleration()));
        }
        Trajectory254<TimedState<S>> traj = new Trajectory254<TimedState<S>>(waypoints);
        traj.setDefaultVelocity(defaultVelocity);
        return traj;
    }

    public static <S extends IPose2d<S>> Trajectory254<TimedState<S>> mirrorAboutYTimed(final Trajectory254<TimedState<S>> trajectory, double yValue, double defaultVelocity) {
        List<TimedState<S>> waypoints = new ArrayList<>(trajectory.length());
        for (int i = 0; i < trajectory.length(); ++i) {
            TimedState<S> timed_state = trajectory.getPoint(i).state();
            waypoints.add(new TimedState<S>(timed_state.state().mirrorAboutY(yValue), timed_state.t(), timed_state.velocity(), timed_state.acceleration()));
        }
        Trajectory254<TimedState<S>> traj = new Trajectory254<TimedState<S>>(waypoints);
        traj.setDefaultVelocity(defaultVelocity);
        return traj;
    }


    public static <S extends IPose2d<S>> Trajectory254<S> transform(final Trajectory254<S> trajectory, Pose2d254 transform) {
        List<S> waypoints = new ArrayList<>(trajectory.length());
        for (int i = 0; i < trajectory.length(); ++i) {
            waypoints.add(trajectory.getPoint(i).state().transformBy(transform));
        }
        return new Trajectory254<>(waypoints);
    }

    /**
     * Creates a Trajectory by sampling a TrajectoryView at a regular interval.
     *
     * @param trajectory_view
     * @param interval
     * @return
     */
    public static <S extends State<S>> Trajectory254<S> resample(
            final TrajectoryView<S> trajectory_view, double interval) {
        if (interval <= Util.kEpsilon) {
            return new Trajectory254<S>();
        }
        final int num_states = (int) Math
                .ceil((trajectory_view.last_interpolant() - trajectory_view.first_interpolant()) / interval);
        ArrayList<S> states = new ArrayList<S>(num_states);

        for (int i = 0; i < num_states; ++i) {
            states.add(trajectory_view.sample(i * interval + trajectory_view.first_interpolant()).state());
        }
        return new Trajectory254<>(states);
    }

    public static Trajectory254<Pose2dWithMotion> trajectoryFromWaypointsAndHeadings(final List<Pose2d254> waypoints, final List<Rotation2d> headings, double
            maxDx, double maxDy, double maxDTheta) {
        List<QuinticHermitePoseSplineNonholonomic> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new QuinticHermitePoseSplineHolonomic(waypoints.get(i - 1), waypoints.get(i), headings.get(i - 1), headings.get(i)));
        }
        QuinticHermitePoseSplineHolonomic.optimizeSpline(splines);
        return trajectoryFromSplines(splines, maxDx, maxDy, maxDTheta);
    }

    public static Trajectory254<Pose2dWithMotion> trajectoryFromSplines(final List<? extends PoseSpline> splines, double maxDx, double maxDy, double maxDTheta) {
        List<Pose2dWithMotion> points = SplineGenerator.parameterizeSplines(splines, maxDx, maxDy, maxDTheta);
        return new Trajectory254<>(points);
    }
}
