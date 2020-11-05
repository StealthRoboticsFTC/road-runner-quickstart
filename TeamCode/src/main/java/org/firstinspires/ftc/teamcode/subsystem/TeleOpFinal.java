package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpFinal extends LinearOpMode {
    private WobbleArm wobbleArm;
    private Shooter shooter;
    private Intake intake;

    @Override
    public void runOpMode() {

        wobbleArm = new WobbleArm(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        waitForStart();

        boolean isYDown = false;
        boolean isBDown = false;

        boolean isRBDown = false;
        boolean isLBDown = false;
        boolean hasRBBeenPressed = false;
        boolean hasLBBeenPressed = false;

        while (!isStopRequested()) {
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
