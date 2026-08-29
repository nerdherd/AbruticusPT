package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class NerdyMath {
    /**
     * Re-maps a number from one range to another.
     * 
     * Similar implementation to the arduino 
     * <a href="https://reference.arduino.cc/reference/en/language/functions/math/map/">
     * Math.map()</a> method.
     * 
     * <p>
     * 
     * Example: map(0.75, 0, 1, 1, 0) returns 0.25.
     * 
     * <p>
     * 
     * Does not ensure that a number will stay within the range. 
     * Use {@link #clamp() clamp()} to do so.
     * 
     * @see https://github.com/arduino/ArduinoCore-API/blob/master/api/Common.cpp
     * 
     * @return the re-mapped number
     */
    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    /**
     * Maps x to the range [0, mod]
     */
    public static double posMod(double x, double mod) {
        return ((x % mod) + mod) % mod;
    }

    public static double degreesToRadians(double deg) {
        return deg * Math.PI/180;
    }

    public static double radiansToDegrees(double rad) {
        return rad * 180 / Math.PI;
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    /**
     * Checks if the value is within the range (inclusive)
     */
    public static boolean inRange(double myValue, double min, double max) {
        return (myValue >= min) && (myValue <= max);
    }
    
    /**
     * Checks if the value is within the range (noninclusive)
     */
    public static boolean inRangeOpen(double myValue, double min, double max) {
        return (myValue > min) && (myValue < max);
    }

    public static double deadband(double value, double min, double max) {
        if(inRange(value, min, max)) return 0;
        return value;
    }

    public static double standardDeviation(double[] values) {
        double sum = 0.0, standardDeviation = 0.0;
        
        for(int i = 0; i < values.length; i++) {
            sum += values[i];
        }

        double mean = sum / values.length;

        for (int i = 0; i < values.length; i++) {
            standardDeviation += Math.pow(values[i] - mean, 2);
        }

        return Math.sqrt(standardDeviation / values.length);
    }

    public static boolean withinStandardDeviation(double[] values, int stdevsAway, double newValue) {
        double sum = 0.0;
        
        for(int i = 0; i < values.length; i++) {
            sum += values[i];
        }

        double mean = sum / values.length;
        double stdev = standardDeviation(values);

        if(newValue >= mean - stdevsAway*stdev && newValue <= mean + stdevsAway*stdev) return true;
        return false;
    }

    public static double continousAddAngle(double angle, double angleToAdd) {
        double offset = angle + angleToAdd;
        if(offset > 180) {
            offset -= 180;
            return -180 + offset;
        }
        else if(offset < -180) {
            offset += 180;
            return 180 + offset;
        }
        return angle;
    }

    public static boolean validatePose(Pose3d pose) {
        if (pose.getX() < 0 || pose.getX() > 16.52) return false;
        if (pose.getY() < 0 || pose.getY() > 8.5) return false;
        if (pose.getZ() < -0.2 || pose.getZ() > 0.5) return false;
        return true;
    }
    
    public static boolean isPoseInsideCircleZone(double x0, double y0, double r, double xn, double yn) {
        double a = (xn - x0);
        double b = (yn - y0);
        return (a*a + b*b) < r*r;
    }

    public static double angleToPose(Pose2d from, Pose2d to) {
        return Math.atan2(to.getY() - from.getY(), to.getX() - from.getX());
    }

    public static double deadband(double x, double db) {
        return (Math.abs(x) <= db) ? 0.0 : x;
    }

    /**
     * 
     * @param a 0
     * @param b 1
     * @param t [0, 1]
     * @return
     */
    public static double lerp(double a, double b, double t) {
        return a*(1 - t) + b*t;
    }
}