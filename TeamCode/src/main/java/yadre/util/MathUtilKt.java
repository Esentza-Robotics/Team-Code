package yadre.util;

public class MathUtilKt {
    public static final double EPSILON = 1.0E-6D;

    public static final boolean epsilonEquals(double $this$epsilonEquals, double other) {
        double var4 = $this$epsilonEquals - other;
        return Math.abs(var4) < 1.0E-6D;
    }
}