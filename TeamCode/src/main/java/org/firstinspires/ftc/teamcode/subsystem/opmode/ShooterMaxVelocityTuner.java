package org.firstinspires.ftc.teamcode.subsystem.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

@Autonomous
public class ShooterMaxVelocityTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, drive);

        waitForStart();

        shooter.setPower(1.0);

        ElapsedTime timer = new ElapsedTime();
        while (!isStopRequested() && timer.seconds() < 10.0) {
            idle();
        }

        double maxVel = shooter.getVelocity();
        shooter.setPower(0.0);

        while(!isStopRequested()) {
            telemetry.addData("maxVelocity", maxVel);
            telemetry.update();
        }
    }
}
