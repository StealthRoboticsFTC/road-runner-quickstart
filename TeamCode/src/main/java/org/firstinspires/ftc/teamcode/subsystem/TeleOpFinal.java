package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

        boolean isXButtonDown = false;
        boolean isAButtonDown = false;

        boolean yHasBeenPressed = false;

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

            if (gamepad2.x && !isXButtonDown) {
                switch(wobbleArm.getArmPosition()){
                    case INITIAL:
                    case PICKUP:
                    case DROPOFF:
                        wobbleArm.moveToCarry();
                        break;
                    case CARRY:
                        if (wobbleArm.isGripOpen()){
                            wobbleArm.moveToPickup();
                        }else {
                            wobbleArm.moveToDropOff();
                        }
                        break;
                }
            }

            if (gamepad2.a && !isAButtonDown) {
                if (!wobbleArm.isGripOpen()) {
                    wobbleArm.gripClose();
                } else {
                    wobbleArm.gripOpen();
                }
            }

            if (gamepad2.y && !yHasBeenPressed) {
                shooter.startRampUp();
                yHasBeenPressed = true;
            } else if (gamepad2.y) {
                shooter.stop();
                yHasBeenPressed = false;
            }

            if (gamepad2.right_trigger > 0.0) {
                shooter.fire();
            }

            shooter.update();

            if (gamepad2.right_bumper && !hasRBBeenPressed && !hasLBBeenPressed) {
                intake.startIn();
                hasRBBeenPressed = true;
            } else if(gamepad2.right_bumper && !hasLBBeenPressed) {
                intake.stop();
                hasRBBeenPressed = false;
            }

            if (gamepad2.left_bumper && !hasLBBeenPressed && !hasRBBeenPressed) {
                intake.startOut();
                hasLBBeenPressed = true;
            } else if(gamepad2.left_bumper && !hasRBBeenPressed) {
                intake.stop();
                hasLBBeenPressed = false;
            }

            isXButtonDown = gamepad2.x;
            isAButtonDown = gamepad2.a;

        }
    }
}
