// MathUtilKt.java
package yadre.util;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;

import kotlin.jvm.internal.Intrinsics;


public final class MathUtil {
    @NotNull
    public static final MathUtil INSTANCE;

    @NotNull
    public final List<Double> numericalDerivative(@NotNull List<Double> x, @NotNull List y) {
        Intrinsics.checkNotNullParameter(x, "x");
        Intrinsics.checkNotNullParameter(y, "y");
        List<Double> deriv = new ArrayList<>();
        int i = 2;

        for(int var5 = x.size(); i < var5; ++i) {
            deriv.add((((Number)y.get(i)).doubleValue() - ((Number)y.get(i - 2)).doubleValue()) / (x.get(i) - x.get(i - 2)));
        }

        deriv.add(0, deriv.get(0));
        deriv.add(deriv.get(deriv.size() - 1));
        return deriv;
    }

    private MathUtil() {
    }

    static {
        INSTANCE = new MathUtil();
    }
}

