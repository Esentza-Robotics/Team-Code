package yadre.tuning;

import com.acmerobotics.roadrunner.kinematics.Kinematics;
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
import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.jetbrains.annotations.NotNull;
import yadre.util.MathUtil;


public final class AccelRegression {
    private final List<Double> timeSamples;
    private final List<Double> positionSamples;
    private final List<Double> powerSamples;


    private List<Double> getVelSamples() {
        return MathUtil.INSTANCE.numericalDerivative(this.timeSamples, this.positionSamples);
    }

    private List<Double> getAccelSamples() {
        return MathUtil.INSTANCE.numericalDerivative(this.timeSamples, this.getVelSamples());
    }

    public final void add(double time, double position, double power) {
        this.timeSamples.add(time); 
        this.positionSamples.add(position);
        this.powerSamples.add(power);
    }

    @NotNull
    public final AccelRegression.AccelResult fit(double kV, double kStatic) {
        SimpleRegression accelReg = new SimpleRegression(false);

        Iterable<Pair<Double, Double>> $this$forEach$iv = CollectionsKt.zip(this.getVelSamples(), this.powerSamples);

        ArrayList<Double> destination$iv$iv = (new ArrayList<>(CollectionsKt.collectionSizeOrDefault($this$forEach$iv, 10)));

        for (Object item$iv$iv : $this$forEach$iv) {
            Pair $dstr$vel$power = (Pair) item$iv$iv;
            double vel = ((Number) $dstr$vel$power.component1()).doubleValue();
            double power = ((Number) $dstr$vel$power.component2()).doubleValue();
            double predPower = Kinematics.calculateMotorFeedforward(vel, 0.0D, kV, 0.0D, kStatic);
            Double var22 = power - predPower;
            destination$iv$iv.add(var22);
        }

        $this$forEach$iv = CollectionsKt.zip((destination$iv$iv), this.getAccelSamples());

        for (Object element$iv : $this$forEach$iv) {
            Pair $dstr$accelPower$accel = (Pair) element$iv;
            double accelPower = ((Number) $dstr$accelPower$accel.component1()).doubleValue();
            double accel = ((Number) $dstr$accelPower$accel.component2()).doubleValue();
            accelReg.addData(accel, accelPower);
        }

        double var23 = accelReg.getSlope();
        return new AccelRegression.AccelResult(Math.abs(var23), accelReg.getRSquare());
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
        Throwable var24 = null;

        try {

            var2.println("vel,accel,power");
            Iterable $this$forEach$iv = CollectionsKt.zip((Iterable)CollectionsKt.zip(this.getVelSamples(), this.getAccelSamples()), this.powerSamples);

            for (Object element$iv : $this$forEach$iv) {
                Pair $dstr$pair$power = (Pair) element$iv;
                Pair pair = (Pair) $dstr$pair$power.component1();
                double power = ((Number) $dstr$pair$power.component2()).doubleValue();
                double vel = ((Number) pair.component1()).doubleValue();
                double accel = ((Number) pair.component2()).doubleValue();
                var2.println("" + vel + ',' + accel + ',' + power);
            }

        } catch (Throwable var22) {
            var24 = var22;
            throw var22;
        } finally {
            CloseableKt.closeFinally(var2, var24);
        }
    }
    @JvmOverloads
    public AccelRegression()
    {
        timeSamples = new ArrayList<>();
        positionSamples = new ArrayList<>();
        powerSamples = new ArrayList<>();
    }

    public static final class AccelResult {
        @JvmField
        public final double kA;
        @JvmField
        public final double rSquare;

        public AccelResult(double kA, double rSquare) {
            this.kA = kA;
            this.rSquare = rSquare;
        }
    }
}
