package Esentza;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name = "Autonomie Albastru Warehouse", group = "Autonomii Esentza")
public class WarehouseAutonomyBlue extends LinearOpMode {

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

   public static double LEVEL1_DISTANCE = 7;
   public static double LEVEL2_DISTANCE = 9;
   public static double LEVEL3_DISTANCE = 20;
   public static double LIFT_POWER = 1;
   public static double LIFT_DISTANCE = 22;
   public static double CLAW_POSITION_CLOSED = 0.7;
   public static double CLAW_POSITION = 0.7;
   public static double CLAW_POSITION_OPEN = 1;

   public static double ARM_POSITION_OPEN = 0;
   public static double ARM_POSITION_CLOSED = 0.7;
   public static double ARM_POSITION = 0.7;

   public static double INTAKE_POWER = 2500;

   public enum DRIVE_ENUM {
      TRAJECTORY_0,
      TRAJECTORY_1,
      TRAJECTORY_2,
      TRAJECTORY_3,
      TRAJECTORY_4,
      TRAJECTORY_5,
      TRAJECTORY_6,
      TRAJECTORY_7,
      TRAJECTORY_8,
      TRAJECTORY_9,
      IDLE
   }

   public enum LIFT_ENUM {
      UP,
      DOWN,
      IDLE
   }

   public enum INTAKE_ENUM {
      FORWARD,
      REVERSE,
      IDLE
   }

   LIFT_ENUM liftEnum = LIFT_ENUM.IDLE;
   DRIVE_ENUM driveEnum = DRIVE_ENUM.TRAJECTORY_0;
   INTAKE_ENUM intakeEnum = INTAKE_ENUM.IDLE;

   NanoClock clock = NanoClock.system();

   @Override
   public void runOpMode() {
      CameraStorage.x1 = 30;
      CameraStorage.y1 = 200;

      CameraStorage.x2 = 220;
      CameraStorage.y2 = 200;

      CameraStorage.x3 = 430;
      CameraStorage.y3 = 200;

      Lift lift = new Lift(hardwareMap);
      Intake intake = new Intake(hardwareMap);
      SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

      LIFT_DISTANCE = LEVEL3_DISTANCE;
      CLAW_POSITION = CLAW_POSITION_OPEN;

      initCam();

      lift.setClawPosition(CLAW_POSITION_CLOSED);
      lift.setArmPosition(ARM_POSITION_CLOSED);

      drive.setPoseEstimate(new Pose2d(10, 61, 0));

      TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(10, 61, 0))
              .addDisplacementMarker(() -> liftEnum = LIFT_ENUM.UP)
              .lineTo(new Vector2d(7, 61))
              .addDisplacementMarker(()->lift.setArmPosition(ARM_POSITION_OPEN))
              .splineTo(new Vector2d(-8, 42), Math.toRadians(-90))
              .build();


      TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(trajectory0.end())
              .addDisplacementMarker(1, ()->
              {
                 lift.setClawPosition(CLAW_POSITION_CLOSED);
                 lift.setArmPosition(ARM_POSITION_CLOSED);
                 LIFT_DISTANCE = 8;
                 liftEnum = LIFT_ENUM.UP;
              })
              .splineTo(new Vector2d(7, 61), Math.toRadians(0))
              .addDisplacementMarker(20, ()->
              {
                 LIFT_DISTANCE = 0;
                 liftEnum = LIFT_ENUM.DOWN;
                 intakeEnum = INTAKE_ENUM.FORWARD;
              })
              .lineTo(new Vector2d(52, 61), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .build();


      TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
              .lineTo(new Vector2d(20, 61), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .addDisplacementMarker(5, ()-> intakeEnum = INTAKE_ENUM.REVERSE)
              .addDisplacementMarker(()->
              {
                 lift.setArmPosition(ARM_POSITION_OPEN);
                 LIFT_DISTANCE = LEVEL3_DISTANCE;
                 liftEnum = LIFT_ENUM.UP;
              })
              .splineTo(new Vector2d(-8, 42), Math.toRadians(-90))
              .build();

      TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory2.end())
              .addDisplacementMarker(1, ()->
              {
                 lift.setClawPosition(CLAW_POSITION_CLOSED);
                 lift.setArmPosition(ARM_POSITION_CLOSED);
              })
              .splineTo(new Vector2d(7, 61), Math.toRadians(0))
              .addDisplacementMarker(20, ()->
              {
                 LIFT_DISTANCE = 0;
                 liftEnum = LIFT_ENUM.DOWN;
                 intakeEnum = INTAKE_ENUM.FORWARD;
              })
              .lineTo(new Vector2d(55, 61), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .build();

      TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(trajectory3.end())
              .lineTo(new Vector2d(20, 60), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .addDisplacementMarker(5, ()-> intakeEnum = INTAKE_ENUM.REVERSE)
              .addDisplacementMarker(()->
              {
                 lift.setArmPosition(ARM_POSITION_OPEN);
                 LIFT_DISTANCE = LEVEL3_DISTANCE;
                 liftEnum = LIFT_ENUM.UP;
              })
              .splineTo(new Vector2d(-8, 42), Math.toRadians(-90))
              .build();

      TrajectorySequence trajectory5 = drive.trajectorySequenceBuilder(trajectory4.end())
              .addDisplacementMarker(1, ()->
              {
                 lift.setClawPosition(CLAW_POSITION_CLOSED);
                 lift.setArmPosition(ARM_POSITION_CLOSED);
              })
              .splineTo(new Vector2d(7, 60), Math.toRadians(0))
              .addDisplacementMarker(20, ()->
              {
                 LIFT_DISTANCE = 0;
                 liftEnum = LIFT_ENUM.DOWN;
                 intakeEnum = INTAKE_ENUM.FORWARD;
              })
              .lineTo(new Vector2d(58, 61), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .build();

      TrajectorySequence trajectory6 = drive.trajectorySequenceBuilder(trajectory5.end())
              .lineTo(new Vector2d(20, 61), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .addDisplacementMarker(5, ()-> intakeEnum = INTAKE_ENUM.REVERSE)
              .addDisplacementMarker(()->
              {
                 lift.setArmPosition(ARM_POSITION_OPEN);
                 LIFT_DISTANCE = LEVEL3_DISTANCE;
                 liftEnum = LIFT_ENUM.UP;
              })
              .splineTo(new Vector2d(-8, 42), Math.toRadians(-90))
              .build();

      TrajectorySequence trajectory7 = drive.trajectorySequenceBuilder(trajectory6.end())
              .addDisplacementMarker(1, ()->
              {
                 lift.setClawPosition(CLAW_POSITION_CLOSED);
                 lift.setArmPosition(ARM_POSITION_CLOSED);
              })
              .splineTo(new Vector2d(7, 60), Math.toRadians(0))
              .addDisplacementMarker(20, ()->
              {
                 LIFT_DISTANCE = 0;
                 liftEnum = LIFT_ENUM.DOWN;
                 intakeEnum = INTAKE_ENUM.FORWARD;
              })
              .lineTo(new Vector2d(58, 60))
              .build();

      TrajectorySequence trajectory8 = drive.trajectorySequenceBuilder(trajectory7.end())
              .lineTo(new Vector2d(15, 61))
              .addDisplacementMarker(5, ()-> intakeEnum = INTAKE_ENUM.REVERSE)
              .addDisplacementMarker(()->
              {
                 lift.setArmPosition(ARM_POSITION_OPEN);
                 LIFT_DISTANCE = LEVEL3_DISTANCE;
                 liftEnum = LIFT_ENUM.UP;
              })
              .splineTo(new Vector2d(-8, 42), Math.toRadians(-90))
              .build();

      TrajectorySequence trajectory9 = drive.trajectorySequenceBuilder(trajectory8.end())
              .addDisplacementMarker(1, ()->
              {
                 lift.setClawPosition(CLAW_POSITION_CLOSED);
                 lift.setArmPosition(ARM_POSITION_CLOSED);
              })
              .splineTo(new Vector2d(7, 60), Math.toRadians(0))
              .addDisplacementMarker(20, ()->
              {
                 LIFT_DISTANCE = 0;
                 liftEnum = LIFT_ENUM.DOWN;
                 intakeEnum = INTAKE_ENUM.IDLE;
              })
              .lineTo(new Vector2d(50, 60))
              .build();


      telemetry.addData("Position", pipeline.getAnalysis());
      telemetry.update();

      waitForStart();

      MarkerDeterminationExample.MarkerDeterminationPipeline.SkystonePosition pos = pipeline.getAnalysis();

      if (pos == MarkerDeterminationExample.MarkerDeterminationPipeline.SkystonePosition.LEFT)
         LIFT_DISTANCE = LEVEL1_DISTANCE;


      else if (pos == MarkerDeterminationExample.MarkerDeterminationPipeline.SkystonePosition.CENTER)
         LIFT_DISTANCE = LEVEL2_DISTANCE;

      drive.followTrajectorySequenceAsync(trajectory0);

      while (opModeIsActive() && !isStopRequested()) {

         switch (driveEnum) {
            case TRAJECTORY_0:
               if (!drive.isBusy() && liftEnum == LIFT_ENUM.IDLE) {
                  lift.setClawPosition(CLAW_POSITION_OPEN);
                  sleep(500);
                  drive.followTrajectorySequenceAsync(trajectory1);
                  driveEnum = DRIVE_ENUM.TRAJECTORY_1;
               }
               break;
            case TRAJECTORY_1:
               if (!drive.isBusy()) {
                  lift.setClawPosition(CLAW_POSITION_CLOSED);
                  sleep(500);
                  drive.followTrajectorySequenceAsync(trajectory2);
                  driveEnum = DRIVE_ENUM.TRAJECTORY_2;
               }
               break;
            case TRAJECTORY_2:
               if (!drive.isBusy() && liftEnum == LIFT_ENUM.IDLE) {
                  lift.setClawPosition(CLAW_POSITION_OPEN);
                  sleep(500);
                  drive.followTrajectorySequenceAsync(trajectory3);
                  driveEnum = DRIVE_ENUM.TRAJECTORY_3;
               }
               break;
            case TRAJECTORY_3:
               if (!drive.isBusy()) {
                  lift.setClawPosition(CLAW_POSITION_CLOSED);
                  sleep(500);
                  drive.followTrajectorySequenceAsync(trajectory4);
                  driveEnum = DRIVE_ENUM.TRAJECTORY_4;
               }
               break;
            case TRAJECTORY_4:
               if (!drive.isBusy() && liftEnum == LIFT_ENUM.IDLE) {
                  lift.setClawPosition(CLAW_POSITION_OPEN);
                  sleep(500);
                  drive.followTrajectorySequenceAsync(trajectory5);
                  driveEnum = DRIVE_ENUM.TRAJECTORY_5;
               }
               break;
            case TRAJECTORY_5:
               if (!drive.isBusy()) {
                  lift.setClawPosition(CLAW_POSITION_CLOSED);
                  sleep(500);
                  drive.followTrajectorySequenceAsync(trajectory6);
                  driveEnum = DRIVE_ENUM.TRAJECTORY_6;
               }
               break;
            case TRAJECTORY_6:
               if (!drive.isBusy() && liftEnum == LIFT_ENUM.IDLE) {
                  lift.setClawPosition(CLAW_POSITION_OPEN);
                  sleep(500);
                  drive.followTrajectorySequenceAsync(trajectory9);
                  driveEnum = DRIVE_ENUM.TRAJECTORY_9;
               }
               break;
            case TRAJECTORY_7:
               if (!drive.isBusy()) {
//                  lift.setClawPosition(CLAW_POSITION_CLOSED);
//                  sleep(250);
//                  drive.followTrajectorySequenceAsync(trajectory8);
                  driveEnum = DRIVE_ENUM.IDLE;
               }
               break;
            case TRAJECTORY_8:
               if (!drive.isBusy() && liftEnum == LIFT_ENUM.IDLE) {
                  lift.setClawPosition(CLAW_POSITION_OPEN);
                  sleep(100);
                  drive.followTrajectorySequenceAsync(trajectory9);
                  driveEnum = DRIVE_ENUM.TRAJECTORY_9;
               }
               break;
            case TRAJECTORY_9:
            {
               if (!drive.isBusy() && liftEnum == LIFT_ENUM.IDLE)
                  driveEnum = DRIVE_ENUM.IDLE;
            }
            break;
            case IDLE:
               if (liftEnum == LIFT_ENUM.IDLE)
                  requestOpModeStop();
               break;
         }

         switch (liftEnum) {
            case UP: {
               if (lift.getLiftPosition() >= LIFT_DISTANCE) {
                  lift.setPower(0);
                  liftEnum = LIFT_ENUM.IDLE;
               } else
                  lift.setPower(-LIFT_POWER);
               break;
            }
            case DOWN: {
               if (lift.getLiftPosition() <= LIFT_DISTANCE) {
                  lift.setPower(0);
                  lift.setClawPosition(CLAW_POSITION_OPEN);
                  liftEnum = LIFT_ENUM.IDLE;
               } else
                  lift.setPower(LIFT_POWER);
               break;
            }
            case IDLE:
               break;
         }

         switch (intakeEnum) {
            case FORWARD:
               intake.setVelocity(INTAKE_POWER, DcMotorSimple.Direction.REVERSE);
               break;
            case REVERSE:
               intake.setVelocity(INTAKE_POWER, DcMotorSimple.Direction.FORWARD);
               break;
            case IDLE:
               intake.setVelocity(0, DcMotorSimple.Direction.FORWARD);
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