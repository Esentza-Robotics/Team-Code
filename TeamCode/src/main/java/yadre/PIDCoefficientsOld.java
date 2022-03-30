package yadre;

import kotlin.Metadata;
import kotlin.jvm.JvmField;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

@Metadata(
        mv = {1, 6, 0},
        k = 1,
        d1 = {"\u0000&\n\u0002\u0018\u0002\n\u0002\u0010\u0000\n\u0000\n\u0002\u0010\u0006\n\u0002\b\b\n\u0002\u0010\u000b\n\u0002\b\u0002\n\u0002\u0010\b\n\u0000\n\u0002\u0010\u000e\n\u0000\b\u0086\b\u0018\u00002\u00020\u0001B#\u0012\b\b\u0002\u0010\u0002\u001a\u00020\u0003\u0012\b\b\u0002\u0010\u0004\u001a\u00020\u0003\u0012\b\b\u0002\u0010\u0005\u001a\u00020\u0003¢\u0006\u0002\u0010\u0006J\t\u0010\u0007\u001a\u00020\u0003HÆ\u0003J\t\u0010\b\u001a\u00020\u0003HÆ\u0003J\t\u0010\t\u001a\u00020\u0003HÆ\u0003J'\u0010\n\u001a\u00020\u00002\b\b\u0002\u0010\u0002\u001a\u00020\u00032\b\b\u0002\u0010\u0004\u001a\u00020\u00032\b\b\u0002\u0010\u0005\u001a\u00020\u0003HÆ\u0001J\u0013\u0010\u000b\u001a\u00020\f2\b\u0010\r\u001a\u0004\u0018\u00010\u0001HÖ\u0003J\t\u0010\u000e\u001a\u00020\u000fHÖ\u0001J\t\u0010\u0010\u001a\u00020\u0011HÖ\u0001R\u0012\u0010\u0005\u001a\u00020\u00038\u0006@\u0006X\u0087\u000e¢\u0006\u0002\n\u0000R\u0012\u0010\u0004\u001a\u00020\u00038\u0006@\u0006X\u0087\u000e¢\u0006\u0002\n\u0000R\u0012\u0010\u0002\u001a\u00020\u00038\u0006@\u0006X\u0087\u000e¢\u0006\u0002\n\u0000¨\u0006\u0012"},
        d2 = {"Lyadre/PIDCoefficientsOld;", "", "kP", "", "kI", "kD", "(DDD)V", "component1", "component2", "component3", "copy", "equals", "", "other", "hashCode", "", "toString", "", "Esentza-Code.TeamCode"}
)
public final class PIDCoefficientsOld {
    @JvmField
    public double kP;
    @JvmField
    public double kI;
    @JvmField
    public double kD;

    public PIDCoefficientsOld(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    // $FF: synthetic method
    public PIDCoefficientsOld(double var1, double var3, double var5, int var7) {
        if ((var7 & 1) != 0) {
            var1 = 0.0D;
        }

        if ((var7 & 2) != 0) {
            var3 = 0.0D;
        }

        if ((var7 & 4) != 0) {
            var5 = 0.0D;
        }
    }

    public PIDCoefficientsOld() {
        this(0.0D, 0.0D, 0.0D, 7);
    }

    public final double component1() {
        return this.kP;
    }

    public final double component2() {
        return this.kI;
    }

    public final double component3() {
        return this.kD;
    }

    @NotNull
    public final PIDCoefficientsOld copy(double kP, double kI, double kD) {
        return new PIDCoefficientsOld(kP, kI, kD);
    }

    // $FF: synthetic method
    public static PIDCoefficientsOld copy$default(PIDCoefficientsOld var0, double var1, double var3, double var5, int var7, Object var8) {
        if ((var7 & 1) != 0) {
            var1 = var0.kP;
        }

        if ((var7 & 2) != 0) {
            var3 = var0.kI;
        }

        if ((var7 & 4) != 0) {
            var5 = var0.kD;
        }

        return var0.copy(var1, var3, var5);
    }

    @NotNull
    public String toString() {
        return "PIDCoefficientsOld(kP=" + this.kP + ", kI=" + this.kI + ", kD=" + this.kD + ")";
    }


    public boolean equals(@Nullable Object var1) {
        if (this != var1) {
            if (var1 instanceof PIDCoefficientsOld) {
                PIDCoefficientsOld var2 = (PIDCoefficientsOld)var1;
                if (Double.compare(this.kP, var2.kP) == 0 && Double.compare(this.kI, var2.kI) == 0 && Double.compare(this.kD, var2.kD) == 0) {
                    return true;
                }
            }

            return false;
        } else {
            return true;
        }
    }
}
