//package Esentza;
//
//import static java.lang.Math.abs;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.util.NanoClock;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.stream.CameraStreamClient;
//import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.elevator.Elevator;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//// 0 98
//// 145 98
//// 280 98
//
//@Config
//@Autonomous(name = "Autonomie Rosu Carousel Mare Ambitie", group = "Autonomii Esentza")
//public class MirceaAmbitios extends LinearOpMode {
//
//   public static OpenCvCamera webcam;
//   MarkerDeterminationExample.MarkerDeterminationPipeline pipeline;
//
//   public void initCam() {
//      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//      webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//      pipeline = new MarkerDeterminationExample.MarkerDeterminationPipeline();
//      webcam.setPipeline(pipeline);
//
//      // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
//      // out when the RC activity is in portrait. We do our actual image processing assuming
//      // landscape orientation, though.
//      webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//
//      webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//         @Override
//         public void onOpened() {
//            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            FtcDashboard.getInstance().startCameraStream(webcam, 0);
//            CameraStreamServer.getInstance().setSource(webcam);
//         }
//
//         @Override
//         public void onError(int errorCode) {
//            /*
//             * This will be called if the camera could not be opened
//             */
//         }
//      });
//   }
//
//   public static double LEVEL1_DISTANCE = 7;
//   public static double LEVEL2_DISTANCE = 11;
//   public static double LEVEL3_DISTANCE = 19;
//   public static double LIFT_POWER = 1;
//   public static double LIFT_MARKER_LEVEL = 16;
//   public static double X_COORDINATE_FOR_CAROUSEL = -0.5;
//   public static double Y_COORDINATE_FOR_CAROUSEL = -24.5;
//   public static double CAROUSEL_DEGREE = 45;
//   public static double DUCK_DEGREE = 30;
//   public static double CLAW_POSITION_CLOSED = 0.7;
//   public static double CLAW_POSITION = 0.7;
//   public static double CLAW_POSITION_OPEN = 1;
//   public static double INTAKE_POWER = 10000;
//
//   public static double ARM_POSITION_OPEN = 0;
//   public static double ARM_POSITION_CLOSED = 0.725;
//   public static double ARM_POSITION = 0.725;
//
//   public static double CAROUSEL_POWER = -1;
//
//   public enum DRIVE_ENUM {
//      TRAJECTORY_0,
//      TRAJECTORY_1,
//      TRAJECTORY_2,
//      TRAJECTORY_3,
//      TRAJECTORY_4,
//      TRAJECTORY_5,
//      TRAJECTORY_6,
//      TRAJECTORY_1_BASE,
//      IDLE
//   }
//
//   public enum LIFT_ENUM {
//      LIFTING,
//      RESET,
//      IDLE
//   }
//
//   public enum INTAKE_ENUM {
//      FORWARD,
//      REVERSE,
//      IDLE
//   }
//
//   LIFT_ENUM liftEnum = LIFT_ENUM.IDLE;
//   INTAKE_ENUM intakeEnum = INTAKE_ENUM.IDLE;
//   DRIVE_ENUM driveEnum = DRIVE_ENUM.TRAJECTORY_0;
//
//   public static double maxVel, finalVel, accel, rampTime, sgn, power, startTime, maxPowerTime;
//   NanoClock clock = NanoClock.system();
//
//   @Override
//   public void runOpMode() {
//      maxVel = Elevator.rpmToVelocity(Elevator.getMaxRpm());
//
//      CameraStorage.x1 = 0;
//      CameraStorage.x2 = 145;
//      CameraStorage.x3 = 280;
//
//      CameraStorage.y1 = 98;
//      CameraStorage.y2 = 98;
//      CameraStorage.y3 = 98;
//
//      Lift lift = new Lift(hardwareMap);
//      Intake intake = new Intake(hardwareMap);
//      SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//      LIFT_MARKER_LEVEL = LEVEL3_DISTANCE;
//      CLAW_POSITION = CLAW_POSITION_OPEN;
//
//      initCam();
//
//      lift.setClawPosition(CLAW_POSITION_CLOSED);
//      lift.setArmPosition(ARM_POSITION_CLOSED);
//
//      drive.setPoseEstimate(new Pose2d());
//
//      TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d())
//              .lineToLinearHeading(new Pose2d(X_COORDINATE_FOR_CAROUSEL, Y_COORDINATE_FOR_CAROUSEL, Math.toRadians(CAROUSEL_DEGREE)),
//                      SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//              .addDisplacementMarker(() -> lift.carousel.setPower(CAROUSEL_POWER))
//              .build();
//
//      TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(trajectory0.end())
//              .addDisplacementMarker(() ->
//              {
//                 lift.setCarouselPower(0);
//                 sgn = 1;
//                 startTime = clock.seconds();
//                 finalVel = LIFT_POWER * maxVel;
//                 accel = (finalVel * finalVel) / (2.0 * abs((LIFT_MARKER_LEVEL)));
//                 rampTime = Math.sqrt(2.0 * abs(LIFT_MARKER_LEVEL) / accel);
//                 liftEnum = LIFT_ENUM.LIFTING;
//                 ARM_POSITION = ARM_POSITION_OPEN;
//              })
//              .lineToLinearHeading(new Pose2d(-4, 17, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//              .lineTo(new Vector2d(-17, 23), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//              .build();
//
//      TrajectorySequence trajectory1forBase = drive.trajectorySequenceBuilder(trajectory0.end())
//              .addDisplacementMarker(() ->
//              {
//                 lift.setCarouselPower(0);
//                 sgn = 1;
//                 startTime = clock.seconds();
//                 finalVel = LIFT_POWER * maxVel;
//                 accel = (finalVel * finalVel) / (2.0 * abs((LEVEL2_DISTANCE)));
//                 rampTime = Math.sqrt(2.0 * abs(LEVEL2_DISTANCE) / accel);
//                 liftEnum = LIFT_ENUM.LIFTING;
//                 ARM_POSITION = ARM_POSITION_OPEN;
//              })
//              .lineToLinearHeading(new Pose2d(-4, 17, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//              .build();
//
//      TrajectorySequence trajectory2forBase = drive.trajectorySequenceBuilder(trajectory1forBase.end())
//              .addDisplacementMarker(() ->
//              {
//                 sgn = -1;
//                 startTime = clock.seconds();
//                 finalVel = LIFT_POWER * maxVel;
//                 accel = (finalVel * finalVel) / (2.0 * abs((LEVEL2_DISTANCE - LEVEL1_DISTANCE)));
//                 rampTime = Math.sqrt(2.0 * abs(LEVEL2_DISTANCE - LEVEL1_DISTANCE) / accel);
//                 liftEnum = LIFT_ENUM.LIFTING;
//              })
//              .lineTo(new Vector2d(-13, 23), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//              .build();
//
//      TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
//              .lineTo(new Vector2d(-8, 23))
//              .addDisplacementMarker(() ->
//              {
//                 ARM_POSITION = ARM_POSITION_CLOSED;
//                 lift.setArmPosition(ARM_POSITION);
//                 double currentDistance = lift.elevator.getCurrentHeight();
//                 if (LEVEL2_DISTANCE < currentDistance)
//                 {
//                    sgn = -1;
//                    currentDistance -= 4;
//                 }
//                 else sgn = 1;
//                 startTime = clock.seconds();
//                 finalVel = LIFT_POWER * maxVel;
//                 accel = (finalVel * finalVel) / (2.0 * abs((LEVEL2_DISTANCE - currentDistance)));
//                 rampTime = Math.sqrt(2.0 * abs(LEVEL2_DISTANCE - currentDistance) / accel);
//                 liftEnum = LIFT_ENUM.LIFTING;
//                 lift.setClawPosition(CLAW_POSITION_CLOSED);
//              })
//              .lineToLinearHeading(new Pose2d(3.5, 40, Math.toRadians(90)))
//              .addDisplacementMarker(()->
//              {
//                 maxPowerTime = LIFT_POWER * LEVEL2_DISTANCE / maxVel;
//                 startTime = clock.seconds();
//                 liftEnum = LIFT_ENUM.RESET;
//                 intakeEnum = INTAKE_ENUM.FORWARD;
//              })
//              .lineTo(new Vector2d(5, 82),
//                      SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//              .build();
//
//      TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory2.end())
//              .addDisplacementMarker(() ->
//              {
//                 sgn = 1;
//                 ARM_POSITION = ARM_POSITION_OPEN;
//                 startTime = clock.seconds();
//                 finalVel = LIFT_POWER * maxVel;
//                 accel = (finalVel * finalVel) / (2.0 * abs(LEVEL3_DISTANCE + 3));
//                 rampTime = Math.sqrt(2.0 * abs(LEVEL3_DISTANCE + 3) / accel);
//                 liftEnum = LIFT_ENUM.LIFTING;
//              })
//              .lineTo(new Vector2d(5, 40),
//                      SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//              .addDisplacementMarker(()->intakeEnum = INTAKE_ENUM.IDLE)
//              .lineToLinearHeading(new Pose2d(-16, 20, Math.toRadians(0)))
//              .build();
//
//      TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(trajectory3.end())
//              .addDisplacementMarker(5, ()->
//              {
//                 lift.setArmPosition(0.375);
//                 lift.setClawPosition(CLAW_POSITION_OPEN);
//              })
//              .lineTo(new Vector2d(-17, -3))
//              .addDisplacementMarker(() -> {
//                  maxPowerTime = LIFT_POWER * (lift.elevator.getCurrentHeight() + 3) / maxVel;
//                  startTime = clock.seconds();
//                  liftEnum = LIFT_ENUM.RESET;
//               })
//              .lineTo(new Vector2d(-17, 41),
//                      SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//              .build();
//
//      TrajectorySequence trajectory5 = drive.trajectorySequenceBuilder(trajectory4.end())
//              .addDisplacementMarker(() ->
//              {
//                 CLAW_POSITION = CLAW_POSITION_CLOSED;
//                 lift.setClawPosition(CLAW_POSITION_CLOSED);
//              })
//              .lineTo(new Vector2d(-17, 23))
//              .build();
//
//      TrajectorySequence trajectory6 = drive.trajectorySequenceBuilder(trajectory5.end())
//              .addDisplacementMarker(5, () ->
//              {
//                 lift.setArmPosition(ARM_POSITION_CLOSED);
//                 lift.setArmPosition(CLAW_POSITION_CLOSED);
//                 CLAW_POSITION = CLAW_POSITION_CLOSED;
//              })
//              .lineToLinearHeading(new Pose2d(4.5, 45, Math.toRadians(90)))
//              .addDisplacementMarker(()->
//              {
//                 CLAW_POSITION = CLAW_POSITION_CLOSED;
//                 lift.setClawPosition(CLAW_POSITION_CLOSED);
//                 maxPowerTime = LIFT_POWER * (lift.elevator.getCurrentHeight() + 3) / maxVel;
//                 startTime = clock.seconds();
//                 liftEnum = LIFT_ENUM.RESET;
//              })
//              .lineTo(new Vector2d(4.5, 75))
//              .build();
//
//      telemetry.addData("Position", pipeline.getAnalysis());
//      telemetry.update();
//
//      waitForStart();
//
//      MarkerDeterminationExample.MarkerDeterminationPipeline.SkystonePosition pos = pipeline.getAnalysis();
//
//      if (pos == MarkerDeterminationExample.MarkerDeterminationPipeline.SkystonePosition.LEFT)
//      {
//         DUCK_DEGREE = 117;
//         LIFT_MARKER_LEVEL = LEVEL1_DISTANCE;
//      }
//
//      else if (pos == MarkerDeterminationExample.MarkerDeterminationPipeline.SkystonePosition.CENTER)
//      {
//         DUCK_DEGREE = 112;
//         LIFT_MARKER_LEVEL = LEVEL2_DISTANCE;
//      }
//
//      else DUCK_DEGREE = 100;
//
//      telemetry.addData("Position", pos);
//      telemetry.addData("Ratusca", DUCK_DEGREE);
//      telemetry.update();
//
//      drive.followTrajectorySequenceAsync(trajectory0);
//
//      while (opModeIsActive() && !isStopRequested()) {
//
//         switch (driveEnum) {
//            case TRAJECTORY_0:
//               if (!drive.isBusy()) {
//                  sleep(1750);
//                  if (pos == MarkerDeterminationExample.MarkerDeterminationPipeline.SkystonePosition.LEFT) {
//                     driveEnum = DRIVE_ENUM.TRAJECTORY_1_BASE;
//                     drive.followTrajectorySequenceAsync(trajectory1forBase);
//                  }
//                  else
//                  {
//                     driveEnum = DRIVE_ENUM.TRAJECTORY_1;
//                     drive.followTrajectorySequenceAsync(trajectory1);
//                  }
//               }
//               break;
//            case TRAJECTORY_1_BASE:
//               if (!drive.isBusy()) {
//                  drive.followTrajectorySequenceAsync(trajectory2forBase);
//                  driveEnum = DRIVE_ENUM.TRAJECTORY_1;
//               }
//               break;
//            case TRAJECTORY_1:
//               if (!drive.isBusy() && liftEnum == LIFT_ENUM.IDLE) {
//                  lift.setClawPosition(CLAW_POSITION_OPEN);
//                  drive.followTrajectorySequenceAsync(trajectory2);
//                  driveEnum = DRIVE_ENUM.TRAJECTORY_2;
//               }
//               break;
//            case TRAJECTORY_2:
//               if (!drive.isBusy()) {
//                  lift.setClawPosition(CLAW_POSITION_CLOSED);
//                  sleep(500);
//                  drive.followTrajectorySequenceAsync(trajectory3);
//                  driveEnum = DRIVE_ENUM.TRAJECTORY_3;
//               }
//               break;
//            case TRAJECTORY_3:
//               if (!drive.isBusy() && liftEnum == LIFT_ENUM.IDLE) {
//                  sleep(500);
//                  lift.setClawPosition(CLAW_POSITION_OPEN);
//                  sleep(500);
//                  driveEnum = DRIVE_ENUM.TRAJECTORY_4;
//                  drive.followTrajectorySequenceAsync(trajectory4);
//               }
//               break;
//            case TRAJECTORY_4:
//               if (!drive.isBusy() && liftEnum == LIFT_ENUM.IDLE) {
//                  lift.setClawPosition(CLAW_POSITION_CLOSED);
//                  sleep(500);
//                  startTime = clock.seconds();
//                  finalVel = LIFT_POWER * maxVel;
//                  accel = (finalVel * finalVel) / (2.0 * abs(LEVEL3_DISTANCE + 3));
//                  rampTime = Math.sqrt(2.0 * abs(LEVEL3_DISTANCE + 3) / accel);
//                  liftEnum = LIFT_ENUM.LIFTING;
//
//                  ARM_POSITION = ARM_POSITION_OPEN;
//                  drive.followTrajectorySequenceAsync(trajectory5);
//                  driveEnum = DRIVE_ENUM.TRAJECTORY_5;
//               }
//               break;
//               case TRAJECTORY_5:
//               if (!drive.isBusy() && liftEnum == LIFT_ENUM.IDLE) {
//                  sleep(500);
//                  lift.setClawPosition(CLAW_POSITION_OPEN);
//                  drive.followTrajectorySequenceAsync(trajectory6);
//                  driveEnum = DRIVE_ENUM.TRAJECTORY_6;
//               }
//               break;
//               case TRAJECTORY_6:
//               if (!drive.isBusy()) {
//                  lift.setClawPosition(CLAW_POSITION_CLOSED);
//                  driveEnum = DRIVE_ENUM.IDLE;
//               }
//               break;
//            case IDLE:
//               requestOpModeStop();
//               break;
//         }
//
//         switch (liftEnum) {
//            case LIFTING:
//               if (clock.seconds() - startTime > rampTime) {
//                  lift.setPower(0);
//                  lift.setArmPosition(ARM_POSITION);
//                  liftEnum = LIFT_ENUM.IDLE;
//               } else {
//                  double elapsedTime = clock.seconds() - startTime;
//
//                  double vel = sgn * accel * elapsedTime;
//                  double power;
//                  power = vel / maxVel;
//                  lift.setPower(power);
//               }
//               break;
//            case RESET:
//               if (clock.seconds() - startTime > maxPowerTime)
//               {
//                  lift.setPower(0);
//                  lift.setClawPosition(CLAW_POSITION);
//                  liftEnum = LIFT_ENUM.IDLE;
//               }
//               else
//                  lift.setPower(-0.9);
//               break;
//            case IDLE:
//               break;
//         }
//
//         switch (intakeEnum) {
//            case FORWARD:
//               intake.setVelocity(INTAKE_POWER, DcMotorSimple.Direction.REVERSE);
//               if (intake.distance() < 5)  {
//                  intakeEnum = INTAKE_ENUM.REVERSE;
//               }
//               break;
//            case REVERSE:
//               intake.setVelocity(INTAKE_POWER, DcMotorSimple.Direction.FORWARD);
//               break;
//            case IDLE:
//               intake.setVelocity(0, DcMotorSimple.Direction.FORWARD);
//               break;
//         }
//
//         drive.update();
//         Pose2d poseEstimate = drive.getPoseEstimate();
//
//         telemetry.addData("x", poseEstimate.getX());
//         telemetry.addData("y", poseEstimate.getY());
//         telemetry.addData("heading", poseEstimate.getHeading());
//         telemetry.addData("range", String.format("%.01f cm", intake.distance()));
//         telemetry.update();
//      }
//   }
//}
