package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class TeleOpFinal extends LinearOpMode {
    private WobbleArm wobbleArm;
    private Shooter shooter;
    private Intake intake;

    @Override
    public void runOpMode() {

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightBack");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wobbleArm = new WobbleArm(hardwareMap);
        shooter = new Shooter(hardwareMap, drive);
        intake = new Intake(hardwareMap);

        waitForStart();

        boolean isYDown = false;
        boolean isBDown = false;

        boolean xHasBeenPressed = false;

        boolean hasRBBeenPressed = false;
        boolean hasLBBeenPressed = false;

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            if (gamepad1.b && !isYDown) {
                if (!wobbleArm.isArmDown()) {
                    wobbleArm.moveDown();
                } else {
                    wobbleArm.moveUp();
                }
            }

            if (gamepad1.y && !isBDown) {
                if (!wobbleArm.isGripOpen()) {
                    wobbleArm.gripClose();
                } else {
                    wobbleArm.gripOpen();
                }
            }

            if (gamepad1.x && !xHasBeenPressed) {
                shooter.startRampUp();
                xHasBeenPressed = true;
            } else if (gamepad1.x) {
                shooter.stop();
                xHasBeenPressed = false;
            }

            if (gamepad1.a) {
                shooter.fire();
            }

            shooter.update();

            if (gamepad1.right_bumper && !hasRBBeenPressed && !hasLBBeenPressed) {
                intake.startIn();
                hasRBBeenPressed = true;
            } else if(gamepad1.right_bumper && !hasLBBeenPressed) {
                intake.stop();
                hasRBBeenPressed = false;
            }

            if (gamepad1.left_bumper && !hasLBBeenPressed && !hasRBBeenPressed) {
                intake.startOut();
                hasLBBeenPressed = true;
            } else if(gamepad1.left_bumper && !hasRBBeenPressed) {
                intake.stop();
                hasLBBeenPressed = false;
            }

            isYDown = gamepad1.y;
            isBDown = gamepad1.b;

        }
    }
}
