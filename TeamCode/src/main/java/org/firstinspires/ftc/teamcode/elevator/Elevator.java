package org.firstinspires.ftc.teamcode.elevator;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import yadre.PIDCoefficientsOld;
import yadre.PIDFControllerOld;

/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */

@Config
public class Elevator {
    private static final double TICKS_PER_REV = 751.8;

    public static double SPOOL_RADIUS = 0.36; // in
    public static double GEAR_RATIO = 0.5; // output (spool) speed / input (motor) speed

    // the operating range of the elevator is restricted to [0, MAX_HEIGHT]
    public static double MAX_HEIGHT = 20; // in

    public static PIDCoefficientsOld PID = new PIDCoefficientsOld(0, 0, 0);

    public static double MAX_VEL = 50; // in/s
    public static double MAX_ACCEL = 30; // in/s^2
    public static double MAX_JERK = 30; // in/s^3

    public static double kV = 0.30973;
    public static double kA = 0.00010;
    public static double kStatic = 0.14421;


    private final DcMotorEx motor;
    private final PIDFControllerOld controller;
    private MotionProfile profile;
    private final NanoClock clock = NanoClock.system();
    private double profileStartTime;
    private final int offset;


    private static double encoderTicksToInches(int ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * SPOOL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return 435;
    }

    public Elevator(HardwareMap hardwareMap) {

        motor = hardwareMap.get(DcMotorEx.class, "lift");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new PIDFControllerOld(PID, kV, kA, kStatic);
        offset = motor.getCurrentPosition();
    }

    public boolean isBusy() {
        return profile != null && (clock.seconds() - profileStartTime) <= profile.duration();
    }

    public double getCurrentHeight() {
        return encoderTicksToInches(motor.getCurrentPosition() - offset);
    }

    public void update() {
        double power;
        double currentHeight = getCurrentHeight();
        if (isBusy()) {
            double time = clock.seconds() - profileStartTime;
            MotionState state = profile.get(time);
            controller.setTargetPosition(state.getX());
            power = controller.update(currentHeight, state.getV(), state.getA());
        } else {
            double desiredHeight = 0;
            controller.setTargetPosition(desiredHeight);
            power = controller.update(currentHeight);
        }
        setPower(power);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }
}