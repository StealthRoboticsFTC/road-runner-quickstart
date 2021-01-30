package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class MiniShooterTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        double power = 1.0;

        waitForStart();

        while(!isStopRequested()) {
            if (gamepad1.dpad_down) power -= 0.002;
            if (gamepad1.dpad_up) power += 0.002;

            shooterMotor.setPower(power);

            telemetry.addData("Powerx1000", power * 1000.0);
            telemetry.addData("Speed", shooterMotor.getVelocity());
            telemetry.update();
        }
    }
}
