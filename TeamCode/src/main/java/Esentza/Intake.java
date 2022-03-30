package Esentza;
import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.TuningController;

class Intake
{
   static DcMotorEx intakeLeft, intakeRight;

   public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(3, 0, 7, 17);

   private final VoltageSensor batteryVoltageSensor;

   public DistanceSensor sensorRange;

   public Intake(HardwareMap hm)
   {
      intakeLeft = hm.get(DcMotorEx.class, "intakeLeft");
      sensorRange = hm.get(DistanceSensor.class, "sensorRange");
      intakeRight = hm.get(DcMotorEx.class, "intakeRight");
      intakeRight.setDirection(DcMotorEx.Direction.REVERSE);

      for (LynxModule module : hm.getAll(LynxModule.class)) {
         module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
      }
      MotorConfigurationType motorConfigurationTypeLeft = intakeLeft.getMotorType().clone();
      MotorConfigurationType motorConfigurationTypeRight = intakeRight.getMotorType().clone();

      motorConfigurationTypeLeft.setAchieveableMaxRPMFraction(1.0);
      motorConfigurationTypeRight.setAchieveableMaxRPMFraction(1.0);

      intakeLeft.setMotorType(motorConfigurationTypeLeft);
      intakeRight.setMotorType(motorConfigurationTypeRight);

      intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      batteryVoltageSensor = hm.voltageSensor.iterator().next();
      setPIDFCoefficients(intakeLeft, MOTOR_VELO_PID);
      setPIDFCoefficients(intakeRight, MOTOR_VELO_PID);
   }

   public double distance()
   {
      return sensorRange.getDistance(DistanceUnit.CM);
   }

   private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
      Log.i("config", "setting custom gains");
      motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
              coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()));
   }


   public double getMeasuredVelocity()
   {
      return intakeRight.getVelocity();
   }

   public void setVelocity(double power, DcMotor.Direction dir) {

      if (dir == DcMotor.Direction.FORWARD)
      {
         intakeRight.setDirection(DcMotor.Direction.FORWARD);
         intakeLeft.setDirection(DcMotor.Direction.REVERSE);
      }
      else {
         intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
         intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
      }

      intakeRight.setVelocity(TuningController.rpmToTicksPerSecond(power));
      intakeLeft.setVelocity(TuningController.rpmToTicksPerSecond(power));

      Log.i("mode", "setting velocity");
   }
}

