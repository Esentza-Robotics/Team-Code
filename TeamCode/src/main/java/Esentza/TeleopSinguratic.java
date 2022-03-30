package Esentza;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name="TeleopSinguratic", group="Linear Opmode")

public class TeleopSinguratic extends LinearOpMode {

   SampleMecanumDrive drive;
   Intake in;
   Lift lift;

   public static double BOX_POSITION = 0.5;
   public static double ARM_POSITION = 0.4;


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

      lift.setArmPosition(ARM_POSITION);
      lift.setClawPosition(0);

      while (opModeIsActive())
      {
         double leftPower = -gamepad1.left_stick_y;
         double rightPower = -gamepad1.right_stick_y;

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

         if (gamepad1.right_bumper)
            in.setVelocity(10000, DcMotor.Direction.REVERSE);

         else in.setVelocity(0, DcMotor.Direction.FORWARD);

         if (gamepad1.dpad_up)
            lift.setPower(0.8);

         else if (gamepad1.dpad_down)
            lift.setPower(-0.8);

         else lift.setPower(0);


      }
   }
}
