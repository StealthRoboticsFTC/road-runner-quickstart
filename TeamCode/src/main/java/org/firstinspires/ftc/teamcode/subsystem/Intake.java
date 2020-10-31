package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private HardwareMap hardwareMap;

    private static double INTAKE_IN_POWER = 1.0;
    private static double INTAKE_OUT_POWER = -1.0;
    private static double INTAKE_STOP_POWER = 0.0;

    public Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        this.rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        this.stop();
    }

    public void startIn() {
        leftIntake.setPower(INTAKE_IN_POWER);
        rightIntake.setPower(INTAKE_IN_POWER);
    }

    public void startOut() {
        leftIntake.setPower(INTAKE_OUT_POWER);
        rightIntake.setPower(INTAKE_OUT_POWER);
    }

    public void stop() {
        leftIntake.setPower(INTAKE_STOP_POWER);
        rightIntake.setPower(INTAKE_STOP_POWER);
    }

}
