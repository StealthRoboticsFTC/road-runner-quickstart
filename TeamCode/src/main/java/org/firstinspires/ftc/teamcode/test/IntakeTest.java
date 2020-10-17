package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        DcMotor frontRight = hardwareMap.get(DcMotor.class, "leftIntake");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "rightIntake");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        while(opModeIsActive()) {
            double power;
            if (gamepad1.right_trigger > 0) {
                while (gamepad1.right_trigger != 0) {
                    power = gamepad1.right_trigger;
                    frontRight.setPower(power);
                    frontLeft.setPower(power);

                }
            }
        }
    }

}