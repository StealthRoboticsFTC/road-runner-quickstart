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

        aim = new AutonomousAiming(hardwareMap, drive, shooter);

        waitForStart();

        boolean isXButtonDown = false;
        boolean isAButtonDown = false;

        boolean yHasBeenPressed = false;

        boolean hasRBBeenPressed = false;
        boolean hasLBBeenPressed = false;
        boolean isDpadDownDown = false;

        while (!isStopRequested()) {
            drive.update();
            shooter.update();
            aim.update();

            telemetry.addData("flyvelo", shooter.getVelocity());
            telemetry.update();

            if (aim.getState() == AutonomousAiming.State.AIMING_POWERSHOT || aim.getState() == AutonomousAiming.State.SHOOTING_POWERSHOT || aim.getState() == AutonomousAiming.State.WAITING_POWERSHOT) {
                shooter.setFlapPosition(0.63);
            } else {
                shooter.setFlapPosition(0.75);
//                shooter.setFlapPosition(0.57);
            }

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
            if (gamepad1.left_trigger > 0.0) {
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

            if (gamepad1.x && !isXButtonDown) {
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

            if (gamepad1.a && !isAButtonDown) {
                if (wobbleArm.isGripOpen()) {
                    wobbleArm.gripClose();
                } else {
                    wobbleArm.gripOpen();
                }
            }

            if (gamepad1.y && !yHasBeenPressed) {
                if (shooter.getState() == Shooter.State.OFF) {
                    shooter.startRampUp();
                } else {
                    shooter.stop();
                }
            }

            if (gamepad1.right_trigger > 0.0 && aim.getState() == AutonomousAiming.State.OFF) {
                aim.startHighGoal();
            }

            if (gamepad1.left_trigger > 0.0 && shooter.getState() != Shooter.State.FIRING) {
                shooter.fire(1);
            }

            if (gamepad1.right_bumper && !hasRBBeenPressed) {
                if (intake.getState() == Intake.State.OFF) {
                    shooter.stop();
                    intake.startIn();
                } else {
                    intake.stop();
                }
            }

            if (gamepad1.left_bumper && !hasLBBeenPressed) {
                if (intake.getState() == Intake.State.OFF) {
                    intake.startOut();
                } else {
                    intake.stop();
                }
            }

//            if (gamepad1.dpad_down) {
//                drive.setPoseEstimate(CALIBRATION_POSE);
//            }

            if (gamepad1.dpad_down && !isDpadDownDown) {
                if (intake.isStackArmOut()) {
                    intake.moveStackArmIn();
                } else {
                    intake.moveStackArmOut();
                }
            }

            isXButtonDown = gamepad1.x;
            isAButtonDown = gamepad1.a;
            yHasBeenPressed = gamepad1.y;
            hasRBBeenPressed = gamepad1.right_bumper;
            hasLBBeenPressed = gamepad1.left_bumper;
            isDpadDownDown = gamepad1.dpad_down;
        }
    }
}
