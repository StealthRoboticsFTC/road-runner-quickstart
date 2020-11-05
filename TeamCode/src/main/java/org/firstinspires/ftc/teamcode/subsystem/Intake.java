package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private Servo vexMotor;
    private HardwareMap hardwareMap;

    private static double INTAKE_IN_POWER = 1.0;
    private static double INTAKE_OUT_POWER = -1.0;
    private static double INTAKE_STOP_POWER = 0.0;

    public Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        this.rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        this.vexMotor = hardwareMap.get(Servo.class, "vexMotor");
        this.stop();
    }

    public void startIn() {
        leftIntake.setPower(INTAKE_IN_POWER);
        rightIntake.setPower(INTAKE_IN_POWER);
        vexMotor.setDirection(Servo.Direction.REVERSE);
    }

    public void startOut() {
        leftIntake.setPower(INTAKE_OUT_POWER);
        rightIntake.setPower(INTAKE_OUT_POWER);
        vexMotor.setDirection(Servo.Direction.FORWARD);
    }

    public void stop() {
        leftIntake.setPower(INTAKE_STOP_POWER);
        rightIntake.setPower(INTAKE_STOP_POWER);
        vexMotor.setPosition(0.0);
    }

}
