package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class TeleopTest extends LinearOpMode {

    static final double OUT_INTAKE_POWER = -0.125;
    static final int MILLIS_BUILD_UP = 1000;
    static final double SHOOTER_ARM_POSITION = 0.5;

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
        Servo shooterArm = hardwareMap.get(Servo.class, "shooterArm");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        long startingTime = 0;
        int shooterArmRuns = 0;
        double intakePower;
        boolean xIsActivated = false;
        boolean powerUp = false;
        double shooterPower;
        boolean fireShooter = false;
        long timeDifference;

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
                startingTime = System.currentTimeMillis();
                xIsActivated = true;
                powerUp = true;
            }

            else if (gamepad2.x) {
                frontShooter.setPower(0);
                backShooter.setPower(0);
                xIsActivated = false;
            }

            timeDifference = System.currentTimeMillis() - startingTime;

            if (powerUp && timeDifference < MILLIS_BUILD_UP) {
                shooterPower = timeDifference / 10.0;
                frontShooter.setPower(shooterPower);
                backShooter.setPower(shooterPower);
            }

            else if (timeDifference >= MILLIS_BUILD_UP) powerUp = false;
             if (gamepad2.a) {
                fireShooter = true;
            }

            if (shooterArmRuns < 3 && fireShooter) {
                shooterArm.setPosition(SHOOTER_ARM_POSITION);
                shooterArm.setPosition(0);
                shooterArmRuns++;
            }

            else if (fireShooter) {
                shooterArmRuns = 0;
                fireShooter = false;
            }
        }
    }

}