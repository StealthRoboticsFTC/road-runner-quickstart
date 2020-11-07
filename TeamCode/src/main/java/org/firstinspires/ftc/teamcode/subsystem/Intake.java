package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private CRServo vexMotor;
    private HardwareMap hardwareMap;

    private static double INTAKE_IN_POWER = -1.0;
    private static double INTAKE_OUT_POWER = 1.0;
    private static double INTAKE_STOP_POWER = 0.0;

    private static double INTAKE_VEX_IN_POWER = -0.5;
    private static double INTAKE_VEX_OUT_POWER = 0.5;
    private static double INTAKE_VEX_STOP_POWER = 0.0;

    public Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        this.rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        this.vexMotor = hardwareMap.get(CRServo.class, "vexMotor");
        this.stop();
    }

    public void startIn() {
        leftIntake.setPower(INTAKE_IN_POWER);
        rightIntake.setPower(INTAKE_IN_POWER);
        vexMotor.setPower(INTAKE_IN_POWER);
    }

    public void startOut() {
        leftIntake.setPower(INTAKE_OUT_POWER);
        rightIntake.setPower(INTAKE_OUT_POWER);
        vexMotor.setPower(INTAKE_VEX_OUT_POWER);
    }

    public void stop() {
        leftIntake.setPower(INTAKE_STOP_POWER);
        rightIntake.setPower(INTAKE_STOP_POWER);
        vexMotor.setPower(INTAKE_VEX_STOP_POWER);
    }

}
