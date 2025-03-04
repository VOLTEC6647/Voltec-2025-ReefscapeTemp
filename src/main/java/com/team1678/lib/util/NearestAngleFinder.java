package com.team1678.lib.util;

import com.team6647.FieldLayout.CoralTarget;

public class NearestAngleFinder {
    public static double findNearestAngle(CoralTarget[] angles, double target) {
        double nearest = angles[0].angle;  // Assume the first angle is the closest
        double minDifference = Math.abs(target - nearest);

        for (CoralTarget angle : angles) {
            double difference = Math.abs(target - angle.angle);
            if (difference < minDifference) {
                minDifference = difference;
                nearest = angle.angle;
            }
        }

        return nearest;
    }
}
