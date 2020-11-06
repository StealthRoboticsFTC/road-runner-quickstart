package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

        boolean isRBDown = false;
        boolean isLBDown = false;
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

            if (gamepad2.x)

            if (gamepad2.right_bumper && !hasRBBeenPressed && !isLBDown) {
                intake.startIn();
                hasRBBeenPressed = true;
            } else if(gamepad2.right_bumper && !isLBDown) {
                intake.stop();
                hasRBBeenPressed = false;
            }

            if (gamepad2.left_bumper && !hasLBBeenPressed && !isRBDown) {
                intake.startOut();
                hasLBBeenPressed = true;
            } else if(gamepad2.left_bumper && !isRBDown) {
                intake.stop();
                hasLBBeenPressed = false;
            }

            isYDown = gamepad1.y;
            isBDown = gamepad1.b;



            isRBDown = gamepad2.right_bumper;
            isLBDown = gamepad2.left_bumper;

        }
    }
}
