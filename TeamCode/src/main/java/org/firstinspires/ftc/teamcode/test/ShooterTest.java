package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class ShooterTest extends LinearOpMode {

    static final int MILLIS_BUILD_UP = 1000;

    @Override
    public void runOpMode() {
        waitForStart();

        DcMotor backShooter = hardwareMap.get(DcMotor.class, "backShooter");
        DcMotor frontShooter = hardwareMap.get(DcMotor.class, "frontShooter");
        Servo flipper = hardwareMap.get(Servo.class, "flipper");

        backShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        flipper.setPosition(0.0);

        boolean xIsActivated = false;

        while(opModeIsActive()) {
            double shooterPower;

            while (gamepad1.right_trigger > 0) {
                shooterPower = gamepad1.right_trigger;
                backShooter.setPower(shooterPower);
                frontShooter.setPower(shooterPower);
            }

            if (gamepad1.x && ! xIsActivated) {
                long startingTime = System.currentTimeMillis();

                while(System.currentTimeMillis() - startingTime < MILLIS_BUILD_UP) {
                    shooterPower = (System.currentTimeMillis() - startingTime) / 10.0;
                    frontShooter.setPower(shooterPower);
                    backShooter.setPower(shooterPower);
                }
                xIsActivated = true;
            }

            else if (gamepad1.x) {
                frontShooter.setPower(0);
                backShooter.setPower(0);
                xIsActivated = false;
            }
        }
    }
}