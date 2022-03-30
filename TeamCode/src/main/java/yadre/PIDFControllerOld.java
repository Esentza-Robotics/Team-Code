package yadre;

import com.acmerobotics.roadrunner.util.MathUtilKt;
import com.acmerobotics.roadrunner.util.NanoClock;
import kotlin.Metadata;
import kotlin.jvm.JvmOverloads;
import kotlin.jvm.functions.Function1;
import kotlin.jvm.internal.Intrinsics;
import org.jetbrains.annotations.NotNull;

public final class PIDFControllerOld {
    private double errorSum;
    private double lastUpdateTimestamp;
    private boolean inputBounded;
    private double minInput;
    private double maxInput;
    private boolean outputBounded;
    private double minOutput;
    private double maxOutput;
    private double targetPosition;
    private double lastError;
    private PIDCoefficientsOld pidOld;
    private double kV;
    private double kA;
    private double kStatic;
    private Function1 kF;
    private NanoClock clock;

    public final double getTargetPosition() {
        return this.targetPosition;
    }

    public final void setTargetPosition(double var1) {
        this.targetPosition = var1;
    }

    public final double getLastError() {
        return this.lastError;
    }

    public final void setInputBounds(double min, double max) {
        if (min < max) {
            this.inputBounded = true;
            this.minInput = min;
            this.maxInput = max;
        }

    }

    public final void setOutputBounds(double min, double max) {
        if (min < max) {
            this.outputBounded = true;
            this.minOutput = min;
            this.maxOutput = max;
        }

    }

    private final double getError(double position) {
        double error = this.targetPosition - position;
        if (this.inputBounded) {
            for(double inputRange = this.maxInput - this.minInput; Math.abs(error) > inputRange / 2.0D; error -= Math.signum(error) * inputRange) {
            }
        }

        return error;
    }

    @JvmOverloads
    public final double update(double position, double velocity, double acceleration) {
        double currentTimestamp = this.clock.seconds();
        double error = this.getError(position);
        double dt = this.lastUpdateTimestamp;
        double var10000;
        if (Double.isNaN(dt)) {
            this.lastError = error;
            this.lastUpdateTimestamp = currentTimestamp;
            var10000 = 0.0D;
        } else {
            dt = currentTimestamp - this.lastUpdateTimestamp;
            this.errorSum += 0.5D * (error + this.lastError) * dt;
            double errorDeriv = (error - this.lastError) / dt;
            this.lastError = error;
            this.lastUpdateTimestamp = currentTimestamp;
            double baseOutput = this.pidOld.kP * error + this.pidOld.kI * this.errorSum + this.pidOld.kD * (errorDeriv - velocity) + this.kV * velocity + this.kA * acceleration + ((Number)this.kF.invoke(position)).doubleValue();
            double output = MathUtilKt.epsilonEquals(baseOutput, 0.0D) ? 0.0D : baseOutput + Math.signum(baseOutput) * this.kStatic;
            if (this.outputBounded) {
                double var19 = this.minOutput;
                double var21 = this.maxOutput;
                var21 = Math.min(output, var21);
                var10000 = Math.max(var19, var21);
            } else {
                var10000 = output;
            }
        }

        return var10000;
    }

    // $FF: synthetic method
    public static double update$default(PIDFControllerOld var0, double var1, double var3, double var5, int var7, Object var8) {
        if ((var7 & 2) != 0) {
            var3 = 0.0D;
        }

        if ((var7 & 4) != 0) {
            var5 = 0.0D;
        }

        return var0.update(var1, var3, var5);
    }

    @JvmOverloads
    public final double update(double position, double velocity) {
        return update$default(this, position, velocity, 0.0D, 4, (Object)null);
    }

    @JvmOverloads
    public final double update(double position) {
        return update$default(this, position, 0.0D, 0.0D, 6, (Object)null);
    }

    public final void reset() {
        this.errorSum = 0.0D;
        this.lastError = 0.0D;
        this.lastUpdateTimestamp = Double.NaN;
    }

    @JvmOverloads
    public PIDFControllerOld(@NotNull PIDCoefficientsOld pidOld, double kV, double kA, double kStatic, @NotNull Function1 kF, @NotNull NanoClock clock) {
        Intrinsics.checkNotNullParameter(pidOld, "pidOld");
        Intrinsics.checkNotNullParameter(kF, "kF");
        Intrinsics.checkNotNullParameter(clock, "clock");

        this.pidOld = pidOld;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.kF = kF;
        this.clock = clock;
        this.lastUpdateTimestamp = Double.NaN;
    }

    // $FF: synthetic method
    public PIDFControllerOld(PIDCoefficientsOld var1, double var2, double var4, double var6, Function1 var8, NanoClock var9, int var10) {
        if ((var10 & 2) != 0) {
            var2 = 0.0D;
        }

        if ((var10 & 4) != 0) {
            var4 = 0.0D;
        }

        if ((var10 & 8) != 0) {
            var6 = 0.0D;
        }

        if ((var10 & 32) != 0) {
            var9 = NanoClock.Companion.system();
        }
    }

    @JvmOverloads
    public PIDFControllerOld(@NotNull PIDCoefficientsOld pidOld, double kV, double kA, double kStatic, @NotNull Function1 kF) {
        this(pidOld, kV, kA, kStatic, kF, (NanoClock)null, 32);
    }

    @JvmOverloads
    public PIDFControllerOld(@NotNull PIDCoefficientsOld pidOld, double kV, double kA, double kStatic) {
        this(pidOld, kV, kA, kStatic, (Function1)null, (NanoClock)null, 48);
    }

    @JvmOverloads
    public PIDFControllerOld(@NotNull PIDCoefficientsOld pidOld, double kV, double kA) {
        this(pidOld, kV, kA, 0.0D, (Function1)null, (NanoClock)null, 56);
    }

    @JvmOverloads
    public PIDFControllerOld(@NotNull PIDCoefficientsOld pidOld, double kV) {
        this(pidOld, kV, 0.0D, 0.0D, (Function1)null, (NanoClock)null, 60);
    }

    @JvmOverloads
    public PIDFControllerOld(@NotNull PIDCoefficientsOld pidOld) {
        this(pidOld, 0.0D, 0.0D, 0.0D, (Function1)null, (NanoClock)null, 62);
    }
}
