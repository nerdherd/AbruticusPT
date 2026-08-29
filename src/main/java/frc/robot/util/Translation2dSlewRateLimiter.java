package frc.robot.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Translation2d;

public class Translation2dSlewRateLimiter {
    private final double rateLimit;
    private Translation2d prevVal;
    private double prevTime;

    public Translation2dSlewRateLimiter(double rateLimit, Translation2d initial) {
        this.rateLimit = rateLimit;
        prevVal = initial;
        prevTime = MathSharedStore.getTimestamp();
    }

    public Translation2dSlewRateLimiter(double rateLimit) {
        this(rateLimit, Translation2d.kZero);
    }

    public Translation2d calculate(Translation2d input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - prevTime;
        prevTime = currentTime;
        Translation2d diff = input.minus(prevVal);
        if (diff.equals(Translation2d.kZero)) return input;
        diff = diff.times(Math.min(rateLimit * elapsedTime / diff.getNorm(), 1.0));
        prevVal = prevVal.plus(diff);
        return prevVal;
    }

    public Translation2d lastValue() {
        return prevVal;
    }

    public void reset(Translation2d value) {
        prevVal = value;
        prevTime = MathSharedStore.getTimestamp();
    }

    public void reset() {
        reset(Translation2d.kZero);
    }
}