package Esentza;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.elevator.ElevatorClassMircea;

@Config
@TeleOp(name="Teleop", group="Linear Opmode")

public class Teleop extends LinearOpMode {

   SampleMecanumDrive drive;
   Intake in;
   Lift lift;

   ElevatorClassMircea elevator;

    public static double CLAW_POSITION_CLOSED = 0.7;
    public static double POWER_AMPLIFICATOR = 0.7;
    public static double CLAW_POSITION_OPEN = 1;
    public static double ARM_POSITION_CLOSED = 0;
    public static double ARM_POSITION_OPEN = 0.725;
    public static double INTAKE_POWER = 2500;


   @Override
   public void runOpMode() {

      drive = new SampleMecanumDrive(hardwareMap);
      in = new Intake(hardwareMap);
      lift = new Lift(hardwareMap);
      elevator = new ElevatorClassMircea(hardwareMap);
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
         double leftPower = -gamepad1.left_stick_y * POWER_AMPLIFICATOR;
         double rightPower = -gamepad1.right_stick_y * POWER_AMPLIFICATOR;

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

         if (gamepad1.x)
             POWER_AMPLIFICATOR = 1;

         if (gamepad1.y)
             POWER_AMPLIFICATOR = 0.7;

         if (gamepad1.b)
             POWER_AMPLIFICATOR = 0.5;

         if (gamepad2.right_bumper)
             in.setVelocity(INTAKE_POWER, DcMotor.Direction.FORWARD);

         else if (gamepad2.left_bumper)
             in.setVelocity(INTAKE_POWER, DcMotor.Direction.REVERSE);

         else in.setVelocity(0, DcMotor.Direction.FORWARD);

         elevator.setPower(gamepad2.left_stick_y);

         telemetry.addData("Nivel lift", elevator.getLiftPosition());
         telemetry.update();

         lift.setCarouselPower(gamepad2.right_stick_y);

        if (gamepad2.a)
            lift.setArmPosition(ARM_POSITION_OPEN);

        if (gamepad2.b)
            lift.setArmPosition(ARM_POSITION_CLOSED);

        if (gamepad2.x)
            lift.setClawPosition(CLAW_POSITION_OPEN);

        if (gamepad2.y)
            lift.setClawPosition(CLAW_POSITION_CLOSED);

        if(in.distance() < 5)
        {
            if (gamepad2.right_stick_y == 0)
                lift.setClawPosition(CLAW_POSITION_CLOSED);
        }

        if (gamepad1.dpad_up)
            elevator.runToPosition(15, 1);

      }
   }
}
