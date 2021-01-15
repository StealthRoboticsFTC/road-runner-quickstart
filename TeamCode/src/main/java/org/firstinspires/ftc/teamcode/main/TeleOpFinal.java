package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.AutonomousAiming;
import org.firstinspires.ftc.teamcode.subsystem.AutonomousPowerShot;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.WobbleArm;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

@Config
@TeleOp
public class TeleOpFinal extends LinearOpMode {
    public static Vector2d GOAL_POSITION = new Vector2d(76.0, 36.0);
    public static double K_TURN = 0.4;
    public static double K_TRANSLATION = 0.5;
    public static Pose2d CALIBRATION_POSE = new Pose2d(-63.5, 63.5, Math.PI);

    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    private WobbleArm wobbleArm;
    private Shooter shooter;
    private Intake intake;
    private AutonomousAiming aim;

    private DistanceSensor backSensor;
    private DistanceSensor frontSensor;
    private BNO055IMU imu;

    private double controlScale(double x, double k) {
        return (1.0 - k) * Math.pow(x, 9) + k * x;
    }

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.pose);

        headingController.setInputBounds(-Math.PI, Math.PI);

        wobbleArm = new WobbleArm(hardwareMap);
        shooter = new Shooter(hardwareMap, drive);
        intake = new Intake(hardwareMap);

        backSensor = hardwareMap.get(DistanceSensor.class, "backSensor");
        frontSensor = hardwareMap.get(DistanceSensor.class, "frontSensor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        aim = new AutonomousAiming(hardwareMap, drive, shooter);

        waitForStart();

        boolean isXButtonDown = false;
        boolean isAButtonDown = false;

        boolean yHasBeenPressed = false;

        boolean hasRBBeenPressed = false;
        boolean hasLBBeenPressed = false;

        while (!isStopRequested()) {
            drive.update();
            shooter.update();
            aim.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            Pose2d velocityEstimate = drive.getPoseVelocity();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            Vector2d gamepadDirection = new Vector2d(-gamepad1.left_stick_y, -gamepad1.right_stick_x);
            double gamepadNorm = gamepadDirection.norm();
            Vector2d movementVector = gamepadNorm != 0.0
                    ? gamepadDirection.times(controlScale(gamepadNorm, K_TRANSLATION) / gamepadNorm)
                    : new Vector2d(0.0, 0.0);

            double scaleFactor = gamepad1.left_bumper ? 0.4
                    : wobbleArm.getArmPosition() == WobbleArm.ArmPosition.PICKUP ? 0.5
                    : 1.0;

            double omegaCorrection = 0.0;
            if (gamepad2.b || gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5) {
                double theta = GOAL_POSITION.minus(poseEstimate.vec()).angle();
                headingController.setTargetPosition(theta);

                omegaCorrection = velocityEstimate != null
                        ? headingController.update(poseEstimate.getHeading(), velocityEstimate.getHeading())
                        : headingController.update(poseEstimate.getHeading());
            }

            Pose2d driveVelocity = new Pose2d(
                    movementVector,
                    controlScale(-gamepad1.left_stick_x, K_TURN) + omegaCorrection * kV * TRACK_WIDTH
            ).times(scaleFactor);

            if((driveVelocity.getX() != 0 || driveVelocity.getY() != 0 || driveVelocity.getHeading() != 0) && aim.getState() != AutonomousAiming.State.OFF) {
                aim.stop();
            }

            if(aim.getState() == AutonomousAiming.State.OFF) {
                drive.setWeightedDrivePower(driveVelocity);
            }

            if (gamepad1.b && aim.getState() == AutonomousAiming.State.OFF) {
                aim.startPowershot();
            }

            if (gamepad2.x && !isXButtonDown) {
                switch(wobbleArm.getArmPosition()){
                    case INITIAL:
                    case PICKUP:
                    case DROPOFF:
                        wobbleArm.moveToCarry();
                        break;
                    case CARRY:
                        if (wobbleArm.isGripOpen()) {
                            wobbleArm.moveToPickup();
                        } else {
                            wobbleArm.moveToDropOff();
                        }
                        break;
                }
            }

            if (gamepad2.a && !isAButtonDown) {
                if (wobbleArm.isGripOpen()) {
                    wobbleArm.gripClose();
                } else {
                    wobbleArm.gripOpen();
                }
            }

            if (gamepad2.y && !yHasBeenPressed) {
                if (shooter.getState() == Shooter.State.OFF) {
                    shooter.startRampUp();
                } else {
                    shooter.stop();
                }
            }

            if (gamepad2.right_trigger > 0.0 && shooter.getState() != Shooter.State.FIRING) {
                aim.startHighGoal();
            }

            if (gamepad2.left_trigger > 0.0 && shooter.getState() != Shooter.State.FIRING) {
                shooter.fire(1);
            }

            if (gamepad2.right_bumper && !hasRBBeenPressed) {
                if (intake.getState() == Intake.State.OFF) {
                    intake.startIn();
                } else {
                    intake.stop();
                }
            }

            if (gamepad2.left_bumper && !hasLBBeenPressed) {
                if (intake.getState() == Intake.State.OFF) {
                    intake.startOut();
                } else {
                    intake.stop();
                }
            }

            if (gamepad1.y) {
                drive.setPoseEstimate(CALIBRATION_POSE);
            }

            isXButtonDown = gamepad2.x;
            isAButtonDown = gamepad2.a;
            yHasBeenPressed = gamepad2.y;
            hasRBBeenPressed = gamepad2.right_bumper;
            hasLBBeenPressed = gamepad2.left_bumper;
        }
    }
}
