package Esentza;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name="TeleopArtur", group="Linear Opmode")

public class TeleopArtur extends LinearOpMode {

   SampleMecanumDrive drive;
   Intake in;
   Lift lift;

   public static double CLAW_POSITION_CLOSED = 0.8;
   public static double CLAW_POSITION_OPEN = 1;
   public static double ARM_POSITION_CLOSED = 0;
   public static double ARM_POSITION_OPEN = 0.725;
   public static double INTAKE_POWER = 10000;
   public static double LENGTH = 40;


   @Override
   public void runOpMode() {

      drive = new SampleMecanumDrive(hardwareMap);
      in = new Intake(hardwareMap);
      lift = new Lift(hardwareMap);
      waitForStart();

      while (!opModeIsActive())
      {
         telemetry.addData("Didn't start the match", "F");
         telemetry.update();
      }

      lift.setArmPosition(ARM_POSITION_OPEN);
      lift.setClawPosition(CLAW_POSITION_OPEN);

      while (opModeIsActive())
      {
         double leftPower = -gamepad1.left_stick_y * 0.7;
         double rightPower = -gamepad1.right_stick_y * 0.7;

         drive.setMotorPowers(leftPower, leftPower, rightPower, rightPower);

         if (gamepad1.right_trigger != 0) {
            double strafePower = gamepad1.right_trigger;
            drive.setMotorPowers(strafePower, -strafePower, strafePower, -strafePower);
         }

         if (gamepad1.left_trigger != 0)
         {
            double strafePower = gamepad1.left_trigger;
            drive.setMotorPowers(-strafePower, strafePower, -strafePower, strafePower);
         }

         if (gamepad2.left_bumper)
            in.setVelocity(INTAKE_POWER, DcMotor.Direction.FORWARD);

         else if (gamepad2.right_bumper)
            in.setVelocity(INTAKE_POWER, DcMotor.Direction.REVERSE);

         else in.setVelocity(0, DcMotor.Direction.FORWARD);

         lift.setPower(gamepad2.left_stick_y);

         lift.setCarouselPower(gamepad2.right_stick_y);

         if (gamepad2.b)
            lift.setArmPosition(ARM_POSITION_OPEN);

         if (gamepad2.x)
            lift.setArmPosition(ARM_POSITION_CLOSED);

         if (gamepad2.a)
            lift.setClawPosition(CLAW_POSITION_OPEN);

         if (gamepad2.y)
            lift.setClawPosition(CLAW_POSITION_CLOSED);

         if (in.distance() < 5)
            INTAKE_POWER = 0;

         if (gamepad2.dpad_up)
            INTAKE_POWER = 10000;

         telemetry.addData("range", String.format("%.01f cm", in.distance()));
         telemetry.update();
      }
   }
}
