package com.team1678.lib.util;

import com.team6647.frc2025.FieldLayout.CoralTarget;
import com.team6647.frc2025.subsystems.Superstructure;

public class NearestCoralFinder {
    public static CoralTarget findNearestCoral(CoralTarget[] angles, double target) {
        // Normalize the target angle to the range [0, 360)
        target = ((target % 360) + 360) % 360;

        CoralTarget nearestTarget = angles[0]; // Assume the first angle is the closest
        double minDifference = calculateCircularDifference(target, nearestTarget.angle);

        for (CoralTarget angle : angles) {
            double currentDifference = calculateCircularDifference(target, angle.angle);
            if (currentDifference < minDifference) {
                minDifference = currentDifference;
                nearestTarget = angle;
            }
        }
        return nearestTarget;
    }

    private static double calculateCircularDifference(double a, double b) {
        // Normalize angles to [0, 360) to handle any input ranges
        a = ((a % 360) + 360) % 360;
        b = ((b % 360) + 360) % 360;

        double diff = Math.abs(a - b);
        return Math.min(diff, 360 - diff);
    }

    public static int getCoralIdFromTarget(CoralTarget target) {
        Superstructure s = Superstructure.getInstance();
        for (int i = 0; i < s.angles.length; i++) {
            if (s.angles[i] == target) { // Note: == is reference comparison; ensure this is intended
                return i;
            }
        }
        return 0;
    }
}