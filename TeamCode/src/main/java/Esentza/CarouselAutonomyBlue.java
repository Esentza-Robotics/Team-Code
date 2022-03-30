package Esentza;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.elevator.Elevator;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name = "Autonomie Albastru Carousel", group = "Autonomii Esentza")
public class CarouselAutonomyBlue extends LinearOpMode {

   public static OpenCvCamera webcam;
   MarkerDeterminationExample.MarkerDeterminationPipeline pipeline;

   public static int WIDTH = 864, H = 480;

   public void initCam() {
      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
      pipeline = new MarkerDeterminationExample.MarkerDeterminationPipeline();
      webcam.setPipeline(pipeline);

      webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

      webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
         @Override
         public void onOpened() {
            webcam.startStreaming(WIDTH, H, OpenCvCameraRotation.UPRIGHT);
            FtcDashboard.getInstance().startCameraStream(webcam, 0);
            CameraStreamServer.getInstance().setSource(webcam);
         }

         @Override
         public void onError(int errorCode) {
            /*
             * This will be called if the camera could not be opened
             */
         }
      });
   }

   public static double LEVEL1_DISTANCE = 8.5;
   public static double LEVEL2_DISTANCE = 13;
   public static double LEVEL3_DISTANCE = 19;
   public static double LIFT_POWER = 1;
   public static double LIFT_MARKER_LEVEL = 22;
   public static double CLAW_POSITION_CLOSED = 0.7;
   public static double CLAW_POSITION = 0.7;
   public static double CLAW_POSITION_OPEN = 1;

   public static double ARM_POSITION_OPEN = 0;
   public static double ARM_POSITION_CLOSED = 0.7;
   public static double ARM_POSITION = 0.7;

   public static double CAROUSEL_POWER = 1;

   public enum DRIVE_ENUM {
      TRAJECTORY_0,
      TRAJECTORY_1,
      TRAJECTORY_2,
      TRAJECTORY_3,
      TRAJECTORY_4,
      TRAJECTORY_5,
      IDLE
   }

   public enum LIFT_ENUM {
      LIFTING,
      RESET,
      IDLE
   }

   LIFT_ENUM liftEnum = LIFT_ENUM.IDLE;
   DRIVE_ENUM driveEnum = DRIVE_ENUM.TRAJECTORY_0;

   public static double maxVel, finalVel, accel, rampTime, sgn, power, startTime, maxPowerTime;
   NanoClock clock = NanoClock.system();

   @Override
   public void runOpMode() {
      CameraStorage.x1 = 30;
      CameraStorage.y1 = 200;

      CameraStorage.x2 = 220;
      CameraStorage.y2 = 200;

      CameraStorage.x3 = 430;
      CameraStorage.y3 = 200;

      maxVel = Elevator.rpmToVelocity(Elevator.getMaxRpm());

      Lift lift = new Lift(hardwareMap);
      SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

      LIFT_MARKER_LEVEL = LEVEL3_DISTANCE;
      CLAW_POSITION = CLAW_POSITION_OPEN;

      initCam();

      lift.setClawPosition(CLAW_POSITION_CLOSED);
      lift.setArmPosition(ARM_POSITION_CLOSED);

      drive.setPoseEstimate(new Pose2d(-24, 64, 0));

      TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-24, 64, 0))
              .addDisplacementMarker(()->
              {
                 sgn = 1;
                 startTime = clock.seconds();
                 finalVel = LIFT_POWER * maxVel;
                 accel = (finalVel * finalVel) / (2.0 * abs((LIFT_MARKER_LEVEL)));
                 rampTime = Math.sqrt(2.0 * abs(LIFT_MARKER_LEVEL) / accel);
                 liftEnum = LIFT_ENUM.LIFTING;
                 ARM_POSITION = ARM_POSITION_OPEN;
              })
              .lineTo(new Vector2d(-30, 58))
              .splineTo(new Vector2d(-40, 15), Math.toRadians(0))
             .addDisplacementMarker(100, ()->lift.setArmPosition(0.5))
              .splineTo(new Vector2d(-21, 20), Math.toRadians(45))
              .build();


      TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(trajectory0.end())
              .lineToLinearHeading(new Pose2d(-45, 20, Math.toRadians(195)))
              .addDisplacementMarker(5, () ->
              {
                 lift.setArmPosition(0.5);
                 maxPowerTime = LIFT_POWER * (0.8 * LIFT_MARKER_LEVEL) / maxVel;
                 startTime = clock.seconds();
                 liftEnum = LIFT_ENUM.RESET;
              })
              .lineTo(new Vector2d(-40, 63.5))
              .addDisplacementMarker(()-> lift.setCarouselPower(CAROUSEL_POWER))
              .build();


      TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
              .lineToLinearHeading(new Pose2d(-10, 52, Math.toRadians(0)))
              .build();

      TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory2.end())
              .lineTo(new Vector2d(-5, 62))
              .lineTo(new Vector2d(-36, 62),
                      SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .build();

      TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(trajectory3.end())
              .addDisplacementMarker(()->
              {
                 lift.setArmPosition(0.5);
                 sgn = 1;
                 startTime = clock.seconds();
                 finalVel = LIFT_POWER * maxVel;
                 accel = (finalVel * finalVel) / (2.0 * abs((LEVEL3_DISTANCE + 1)));
                 rampTime = Math.sqrt(2.0 * abs(LEVEL3_DISTANCE + 1) / accel);
                 liftEnum = LIFT_ENUM.LIFTING;
                 ARM_POSITION = ARM_POSITION_OPEN;
              })
              .lineToLinearHeading(new Pose2d(-40, 23, Math.toRadians(90)))
              .lineToLinearHeading(new Pose2d(-21, 20, Math.toRadians(180)))
              .build();

      TrajectorySequence trajectory5 = drive.trajectorySequenceBuilder(trajectory4.end())
              .lineTo(new Vector2d(-48, 20))
              .lineTo(new Vector2d(-50, 40))
              .build();


      telemetry.addData("Position", pipeline.getAnalysis());
      telemetry.update();

      waitForStart();

      MarkerDeterminationExample.MarkerDeterminationPipeline.SkystonePosition pos = pipeline.getAnalysis();

      if (pos == MarkerDeterminationExample.MarkerDeterminationPipeline.SkystonePosition.LEFT)
         LIFT_MARKER_LEVEL = LEVEL1_DISTANCE;


      else if (pos == MarkerDeterminationExample.MarkerDeterminationPipeline.SkystonePosition.CENTER)
         LIFT_MARKER_LEVEL = LEVEL2_DISTANCE;


      drive.followTrajectorySequenceAsync(trajectory0);

      while (opModeIsActive() && !isStopRequested()) {

         switch (driveEnum) {
            case TRAJECTORY_0:
               if (!drive.isBusy() && liftEnum == LIFT_ENUM.IDLE) {
                  sleep(1000);
                  lift.setClawPosition(CLAW_POSITION_OPEN);
                  sleep(500);
                  drive.followTrajectorySequenceAsync(trajectory1);
                  driveEnum = DRIVE_ENUM.TRAJECTORY_1;
               }
               break;
            case TRAJECTORY_1:
               if (!drive.isBusy()) {
                  sleep(2000);
                  lift.setCarouselPower(0);
                  drive.followTrajectorySequenceAsync(trajectory2);
                  driveEnum = DRIVE_ENUM.TRAJECTORY_2;
               }
               break;
            case TRAJECTORY_2:
               if (!drive.isBusy()) {
                  lift.setArmPosition(0.135);
                  sleep(1000);
                  drive.followTrajectorySequenceAsync(trajectory3);
                  driveEnum = DRIVE_ENUM.TRAJECTORY_3;
               }
               break;
               case TRAJECTORY_3:
               if (!drive.isBusy()) {
                  sleep(500);
                  lift.setClawPosition(CLAW_POSITION_CLOSED);
                  sleep(500);
                  drive.followTrajectorySequenceAsync(trajectory4);
                  driveEnum = DRIVE_ENUM.TRAJECTORY_4;
               }
               break;
            case TRAJECTORY_4:
               if (!drive.isBusy() && liftEnum == LIFT_ENUM.IDLE) {
                  lift.setArmPosition(ARM_POSITION_OPEN);
                  sleep(1000);
                  lift.setClawPosition(CLAW_POSITION_OPEN);
                  sleep(500);
                  lift.setArmPosition(ARM_POSITION_CLOSED);
                  drive.followTrajectorySequenceAsync(trajectory5);
                  driveEnum = DRIVE_ENUM.TRAJECTORY_5;
               }
               break;
            case TRAJECTORY_5:
               if (!drive.isBusy()) {
                  lift.setClawPosition(CLAW_POSITION_CLOSED);
                  maxPowerTime = LIFT_POWER * (LEVEL3_DISTANCE - 2) / maxVel;
                  startTime = clock.seconds();
                  liftEnum = LIFT_ENUM.RESET;
                  driveEnum = DRIVE_ENUM.IDLE;
               }
               break;
            case IDLE:
               if (liftEnum == LIFT_ENUM.IDLE)
                  requestOpModeStop();
               break;
         }

         switch (liftEnum) {
            case LIFTING:
               if (clock.seconds() - startTime > rampTime) {
                  lift.setPower(0);
                  lift.setArmPosition(ARM_POSITION);
                  liftEnum = LIFT_ENUM.IDLE;
               } else {
                  double elapsedTime = clock.seconds() - startTime;

                  double vel = sgn * accel * elapsedTime;
                  double power;
                  power = vel / maxVel;
                  lift.setPower(power);
               }
               break;
            case RESET:
               if (clock.seconds() - startTime > maxPowerTime)
               {
                  lift.setPower(0);
                  lift.setClawPosition(CLAW_POSITION);
                  liftEnum = LIFT_ENUM.IDLE;
               }
               else
                  lift.setPower(-0.7);
               break;
            case IDLE:
               break;
         }

         drive.update();
         Pose2d poseEstimate = drive.getPoseEstimate();

         telemetry.addData("x", poseEstimate.getX());
         telemetry.addData("y", poseEstimate.getY());
         telemetry.addData("heading", poseEstimate.getHeading());
         telemetry.update();
      }
   }
}