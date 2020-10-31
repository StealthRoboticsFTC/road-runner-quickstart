package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.WobbleArm;

@TeleOp
public class WobbleArmTest extends LinearOpMode {


    private WobbleArm wobbleArm;

    @Override
    public void runOpMode() {

        wobbleArm = new WobbleArm(hardwareMap);

        Servo arm = hardwareMap.get(Servo.class, "arm");
        Servo pincher = hardwareMap.get(Servo.class, "pincher");

        waitForStart();

        boolean isADown=false;
        boolean isArmUp=false;

        boolean isXDown=false;
        boolean isPincherClosed=false;

        pincher.setPosition(0.0);
        arm.setPosition(0.0);
        while (!isStopRequested()) {
            if (gamepad1.a && !isADown) {

                if (isArmUp){
                    wobbleArm.moveDown();
                } else {
                    wobbleArm.moveUp();
                }
                isArmUp=!isArmUp;

            }
            if (gamepad1.x && !isXDown) {
                if (isPincherClosed){
                    wobbleArm.gripClose();
                }else {
                    wobbleArm.gripOpen();
                }
                isPincherClosed=!isPincherClosed;
            }
            isADown=gamepad1.a;
            isXDown=gamepad1.x;
        }
    }
}
