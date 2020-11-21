package org.firstinspires.ftc.teamcode.subsystem.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

import static org.firstinspires.ftc.teamcode.subsystem.Shooter.TARGET_VELOCITY;

@Config
public class ShooterVelocityPIDTuner extends LinearOpMode {
    public static double HIGH_VELOCITY_FRACTION = 1.0;
    public static double LOW_VELOCITY_FRACTION = 0.7;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, drive);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        shooter.startRampUp();

        while (!isStopRequested() && shooter.getShooterState() != Shooter.State.RUNNING) {
            shooter.update();
        }

        boolean isHigh = true;
        double velocityTarget = HIGH_VELOCITY_FRACTION * TARGET_VELOCITY;

        while (!isStopRequested()) {
            if (timer.seconds() > 1.0) {
                isHigh = !isHigh;
                if (isHigh) {
                    velocityTarget = HIGH_VELOCITY_FRACTION * TARGET_VELOCITY;
                } else {
                    velocityTarget = LOW_VELOCITY_FRACTION * TARGET_VELOCITY;
                }
                shooter.setVelocity(velocityTarget);

                timer.reset();
            }

            shooter.update();

            double velocity = shooter.getVelocity();
            double error = velocity - velocityTarget;

            telemetry.addData("velocity", velocity);
            telemetry.addData("velocityTarget", velocityTarget);
            telemetry.addData("velocityError", error);
            telemetry.update();
        }
    }
}
