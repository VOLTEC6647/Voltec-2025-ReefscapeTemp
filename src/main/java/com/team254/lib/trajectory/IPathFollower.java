package com.team254.lib.trajectory;

import com.team254.lib.geometry.Pose2d254;
import com.team254.lib.geometry.Twist2d;

public interface IPathFollower {
    Twist2d steer(Pose2d254 current_pose);

    boolean isDone();
}
