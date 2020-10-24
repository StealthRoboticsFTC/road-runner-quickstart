package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class TeleopTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();


        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "leftIntake");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "rightIntake");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {
            double forwardPower = -gamepad1.left_stick_y;
            double straife = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            leftFront.setPower(forwardPower + straife + turn);
            leftBack.setPower(forwardPower - straife + turn);
            rightFront.setPower(forwardPower - straife - turn);
            rightBack.setPower(straife + forwardPower - turn);

            double intakePower;
            if (gamepad1.left_trigger > 0) {
                while(gamepad1.left_trigger != 0) {
                    intakePower = gamepad1.left_trigger;
                    frontRight.setPower(intakePower);
                    frontLeft.setPower(intakePower);

                }
            }
        }
    }

}