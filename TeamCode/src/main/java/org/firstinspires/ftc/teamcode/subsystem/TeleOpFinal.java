package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOpFinal extends LinearOpMode {
    private WobbleArm wobbleArm;
    private Shooter shooter;
    private Intake intake;

    @Override
    public void runOpMode() {

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        wobbleArm = new WobbleArm(hardwareMap);
        // shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        waitForStart();

        boolean isYDown = false;
        boolean isBDown = false;

        boolean xHasBeenPressed = false;

        boolean hasRBBeenPressed = false;
        boolean hasLBBeenPressed = false;

        while (!isStopRequested()) {
            double forwardPower = -gamepad1.left_stick_y;
            double straife = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            leftFront.setPower(forwardPower + straife + turn);
            leftBack.setPower(forwardPower - straife + turn);
            rightFront.setPower(forwardPower - straife - turn);
            rightBack.setPower(straife + forwardPower - turn);

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
