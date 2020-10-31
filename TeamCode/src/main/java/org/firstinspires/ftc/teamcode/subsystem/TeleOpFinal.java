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

            isYDown = gamepad1.y;
            isBDown = gamepad1.b;

        }
    }
}
