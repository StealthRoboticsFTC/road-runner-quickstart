package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.WobbleArm;

@TeleOp
public class WobbleArmTest extends LinearOpMode {

    private WobbleArm wobbleArm;

    @Override
    public void runOpMode() {

        wobbleArm = new WobbleArm(hardwareMap);

        waitForStart();

        boolean isADown=false;
        boolean isXDown=false;

        while (!isStopRequested()) {
            if (gamepad1.a && !isADown) {

                if (!wobbleArm.isArmDown()){
                    wobbleArm.moveDown();
                } else {
                    wobbleArm.moveUp();
                }

            }
            if (gamepad1.x && !isXDown) {
                if (!wobbleArm.isGripOpen()){
                    wobbleArm.gripClose();
                }else {
                    wobbleArm.gripOpen();
                }
            }
            isADown=gamepad1.a;
            isXDown=gamepad1.x;
        }
    }
}
