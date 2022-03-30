package Esentza;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import yadre.tuning.AccelRegression;
import yadre.tuning.RampRegression;
import com.acmerobotics.roadrunner.util.NanoClock;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.elevator.Elevator;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
public class Lift{
   public static double TICKS_PER_REV = 8192;
   public static double SPOOL_RADIUS = 1.5; // in
   public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

   private final Encoder encoderLift;

   private DcMotorEx lift;
   Servo arm, claw;
   CRServo carousel;

   public Lift(HardwareMap hm)
   {
      lift = hm.get(DcMotorEx.class, "lift");
      encoderLift = new Encoder(hm.get(DcMotorEx.class, "lift"));


      lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      lift.setDirection(DcMotorSimple.Direction.REVERSE);
      encoderLift.setDirection(Encoder.Direction.REVERSE);

      carousel = hm.get(CRServo.class, "carousel");
      arm = hm.get(Servo.class, "arm");
      claw = hm.get(Servo.class, "claw");
   }

   public void setArmPosition(double position)
   {
      arm.setPosition(position);
   }
   public double getArmPosition()
   {
      return arm.getPosition();
   }

   public void setClawPosition(double position)
   {
      claw.setPosition(position);
   }
   public double getClawPosition()
   {
      return claw.getPosition();
   }

   public void setCarouselPower(double power)
   {
      carousel.setPower(power);
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
