package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoTest extends LinearOpMode {
    public static double servoPosition = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "arm");

        servoPosition = 0.5;

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.dpad_up) {
                servoPosition += 0.0003;
            } else if (gamepad1.dpad_down) {
                servoPosition -= 0.0003;
            }

            servo.setPosition(servoPosition);

            telemetry.addData("position", servoPosition);
            telemetry.update();
        }
    }
}
