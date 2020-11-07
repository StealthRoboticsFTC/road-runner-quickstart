package org.firstinspires.ftc.teamcode.parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config

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
            double intakePower;
            while (gamepad1.left_trigger > 0) {
                intakePower = gamepad1.left_trigger;
                frontRight.setPower(intakePower);
                frontLeft.setPower(intakePower);

            }
        }
    }
}