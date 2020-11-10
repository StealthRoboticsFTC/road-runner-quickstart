package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class test extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor frontShooter = hardwareMap.get(DcMotor.class, "frontShooter");
        DcMotor backShooter = hardwareMap.get(DcMotor.class, "backShooter");
        CRServo conveyor = hardwareMap.get(CRServo.class, "conveyor" );

        waitForStart();
        while (!isStopRequested()){
            double shooterPower = gamepad1.right_trigger;
            double conveyorPower = gamepad1.left_trigger;

            frontShooter.setPower(shooterPower);
            backShooter.setPower(shooterPower);

            conveyor.setPower(conveyorPower * .5);
        }

    }

}
