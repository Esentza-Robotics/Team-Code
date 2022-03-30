package yadre.tuning;

import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.jetbrains.annotations.NotNull;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.io.Writer;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;

import kotlin.Pair;
import kotlin.collections.CollectionsKt;
import kotlin.io.CloseableKt;
import kotlin.jvm.JvmField;
import kotlin.jvm.JvmOverloads;
import kotlin.jvm.internal.Intrinsics;
import kotlin.text.Charsets;
import yadre.util.MathUtil;

public final class RampRegression {
    private List<Double> timeSamples;
    private List<Double> positionSamples;
    private List<Double> powerSamples;

    private List<Double> getVelSamples() {
        return MathUtil.INSTANCE.numericalDerivative(this.timeSamples, this.positionSamples);
    }

    public final void add(double time, double position, double power) {
        this.timeSamples.add(time);
        this.positionSamples.add(position);
        this.powerSamples.add(power);
    }

    @JvmOverloads
    @NotNull
    public final RampRegression.RampResult fit(boolean fitStatic) {
        SimpleRegression rampReg = new SimpleRegression(fitStatic);
        Iterable $this$forEach$iv = CollectionsKt.zip(this.getVelSamples(), this.powerSamples);

        for (Object element$iv : $this$forEach$iv) {
            Pair $dstr$vel$power = (Pair) element$iv;
            double vel = ((Number) $dstr$vel$power.component1()).doubleValue();
            double power = ((Number) $dstr$vel$power.component2()).doubleValue();
            rampReg.addData(vel, power);
        }

        double var13 = rampReg.getSlope();
        double var10002 = Math.abs(var13);
        var13 = rampReg.getIntercept();
        return new RampRegression.RampResult(var10002, Math.abs(var13), rampReg.getRSquare());
    }

    public final void save(@NotNull File file) {
        Intrinsics.checkNotNullParameter(file, "file");
        Charset var3 = Charsets.UTF_8;
        short var6 = 8192;
        OutputStream var10 = null;
        try {
            var10 = new FileOutputStream(file);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        Writer var8 = new OutputStreamWriter(var10, var3);
        PrintWriter var2 = new PrintWriter(new BufferedWriter(var8, var6));
        Throwable var20 = null;

        try {
            var2.println("vel,power");
            Iterable $this$forEach$iv = CollectionsKt.zip(this.getVelSamples(), this.powerSamples);

            for (Object element$iv : $this$forEach$iv) {
                Pair $dstr$vel$power = (Pair) element$iv;
                double vel = ((Number) $dstr$vel$power.component1()).doubleValue();
                double power = ((Number) $dstr$vel$power.component2()).doubleValue();
                var2.println("" + vel + ',' + power);
            }

        } catch (Throwable var18) {
            var20 = var18;
            throw var18;
        } finally {
            CloseableKt.closeFinally(var2, var20);
        }
    }

    // $FF: synthetic method
    @JvmOverloads
    public RampRegression()
    {
        timeSamples = new ArrayList<>();
        positionSamples = new ArrayList<>();
        powerSamples = new ArrayList<>();
    }

    public static final class RampResult {
        @JvmField
        public final double kV;
        @JvmField
        public final double kStatic;
        @JvmField
        public final double rSquare;

        public RampResult(double kV, double kStatic, double rSquare) {
            this.kV = kV;
            this.kStatic = kStatic;
            this.rSquare = rSquare;
        }
        }
    }
