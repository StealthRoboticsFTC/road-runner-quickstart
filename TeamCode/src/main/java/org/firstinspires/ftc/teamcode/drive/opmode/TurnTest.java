package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            drive.turn(Math.toRadians(ANGLE));
            timer.reset();
            while (!isStopRequested() && timer.seconds() < 1.0) {
                drive.update();
            }
        }

    }
}
