package com.team1678.lib.util;

import com.team6647.frc2025.FieldLayout.CoralTarget;
import com.team6647.frc2025.subsystems.Superstructure;

public class NearestCoralFinder {
    public static CoralTarget findNearestCoral(CoralTarget[] angles, double target) {
        double nearest = angles[0].angle;  // Assume the first angle is the closest
        double minDifference = Math.abs(target - nearest);

        for (CoralTarget angle : angles) {
            double difference = Math.abs(target - angle.angle);
            if (difference < minDifference) {
                minDifference = difference;
                nearest = angle.angle;
            }
        }
        for (CoralTarget angle : angles) {
            if (nearest == angle.angle){
                return angle;
            }
        }
        return null;
    }
    public static int getCoralIdFromTarget(CoralTarget target){
        Superstructure s = Superstructure.getInstance();
        for(int i = 0; i < s.angles.length;i++){
            if(s.angles[i] == target){
                return i;
            }
        }
        return 0;
    }
}
