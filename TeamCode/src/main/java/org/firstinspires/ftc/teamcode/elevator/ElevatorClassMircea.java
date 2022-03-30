package org.firstinspires.ftc.teamcode.elevator;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class ElevatorClassMircea {
   public static double TICKS_PER_REV = 8192;
   public static double SPOOL_RADIUS = 1.5; // in
   public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

   private final Encoder encoderLift;

   private DcMotorEx lift;

   public ElevatorClassMircea(HardwareMap hardwareMap)
   {
      lift = hardwareMap.get(DcMotorEx.class, "lift");
      encoderLift = new Encoder(hardwareMap.get(DcMotorEx.class, "lift"));


      lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      lift.setDirection(DcMotorSimple.Direction.REVERSE);
      encoderLift.setDirection(Encoder.Direction.REVERSE);
   }

   public static double encoderTicksToInches(double ticks) {
      return SPOOL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
   }

   public double getLiftPosition() {
      return encoderTicksToInches(encoderLift.getCurrentPosition());
   }

   public double getCorrectedVelocity() {
      return encoderTicksToInches(encoderLift.getCorrectedVelocity());
   }

   public void runToPosition(double distance, double power)
   {
      if (getLiftPosition() < distance)
      {
         lift.setPower(-power);
         while (getLiftPosition() < distance);
         lift.setPower(0);
      }
      else
      {
         lift.setPower(power);
         while (getLiftPosition() > distance);
         lift.setPower(0);
      }
   }

   public void setPower(double power)
   {
      lift.setPower(power);
   }
}