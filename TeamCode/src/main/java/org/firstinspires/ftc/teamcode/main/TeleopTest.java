package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class TeleopTest extends LinearOpMode {

    static final double OUT_INTAKE_POWER = -0.6;
    static final int MILLIS_BUILD_UP = 1000;

    @Override
    public void runOpMode() {
        waitForStart();

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "leftIntake");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "rightIntake");
        DcMotor backShooter = hardwareMap.get(DcMotor.class, "backShooter");
        DcMotor frontShooter = hardwareMap.get(DcMotor.class, "frontShooter");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        double intakePower;
        boolean xIsActivated = false;
        double shooterPower;

        while (opModeIsActive()) {
            double forwardPower = -gamepad1.left_stick_y;
            double straife = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            leftFront.setPower(forwardPower + straife + turn);
            leftBack.setPower(forwardPower - straife + turn);
            rightFront.setPower(forwardPower - straife - turn);
            rightBack.setPower(straife + forwardPower - turn);

            intakePower = gamepad1.left_trigger;
            frontRight.setPower(intakePower);
            frontLeft.setPower(intakePower);

            if (gamepad2.left_bumper) {
                intakePower = OUT_INTAKE_POWER;
                frontRight.setPower(intakePower);
                frontLeft.setPower(intakePower);
            }

            if (gamepad2.x && ! xIsActivated) {
                long startingTime = System.currentTimeMillis();

                while(System.currentTimeMillis() - startingTime < MILLIS_BUILD_UP) {
                    shooterPower = (System.currentTimeMillis() - startingTime) / 10.0;
                    frontShooter.setPower(shooterPower);
                    backShooter.setPower(shooterPower);
                }
                xIsActivated = true;
            }

            else if (gamepad2.x) {
                frontShooter.setPower(0);
                backShooter.setPower(0);
                xIsActivated = false;
            }
        }
    }

}