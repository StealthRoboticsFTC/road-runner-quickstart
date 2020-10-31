package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class RevDebug extends LinearOpMode {
    @Override
    public void runOpMode() {
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        double baseVoltage = voltageSensor.getVoltage();

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            double forwardPower = -gamepad1.left_stick_y;
            double strafePower = gamepad1.left_stick_x;
            double turnPower = gamepad1.right_stick_x;

            leftFront.setPower(forwardPower + strafePower + turnPower);
            leftBack.setPower(forwardPower - strafePower + turnPower);
            rightFront.setPower(forwardPower - strafePower - turnPower);
            rightBack.setPower(forwardPower + strafePower - turnPower);

            // telemetry collection
            double[] currents = new double[] {
                    leftFront.getCurrent(CurrentUnit.AMPS),
                    leftBack.getCurrent(CurrentUnit.AMPS),
                    rightFront.getCurrent(CurrentUnit.AMPS),
                    rightBack.getCurrent(CurrentUnit.AMPS)
            };
            double totalCurrent = 0.0;
            for (Double current : currents) {
                totalCurrent += current;
            }

            double voltage = voltageSensor.getVoltage();

            if (totalCurrent < 0.1) {
                baseVoltage = voltage;
            }

            telemetry.addData("Voltage (v)", voltage);
            telemetry.addData("Amperage (a)",  "total: " + totalCurrent + " (" + currents[0] + " + "
                    + currents[1] + " + " + currents[2] + " + " + currents[3] + ")");
            telemetry.addData("Estimated Battery Internal Resistance (ohm)",
                    (baseVoltage - voltage) / totalCurrent);
            telemetry.addData("Base Voltage (v)", baseVoltage);
            telemetry.update();
        }
    }
}