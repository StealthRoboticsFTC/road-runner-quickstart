package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class VexTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo vex = hardwareMap.get(CRServo.class, "vex");

        waitForStart();

        while (!isStopRequested()) {

        }
    }
}
