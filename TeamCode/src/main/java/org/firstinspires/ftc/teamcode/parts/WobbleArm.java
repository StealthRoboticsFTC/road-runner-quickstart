package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class WobbleArm extends LinearOpMode {

    @Override
    public void runOpMode() {

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
                    arm.setPosition(0.0);
                }else {
                    arm.setPosition(0.5);
                }
                isArmUp=!isArmUp;

            }
            if (gamepad1.x && !isXDown) {
                if (isPincherClosed){
                    pincher.setPosition(0.0);
                }else {
                    pincher.setPosition(0.5);
                }
                isPincherClosed=!isPincherClosed;
            }
            isADown=gamepad1.a;
            isXDown=gamepad1.x;
        }
    }
}
