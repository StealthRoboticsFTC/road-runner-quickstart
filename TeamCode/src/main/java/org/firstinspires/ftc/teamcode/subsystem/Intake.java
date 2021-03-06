package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    public static double ROLLER_IN_POWER = -1.0;
    public static double ROLLER_OUT_POWER = 1.0;
    public static double INTAKE_IN_POWER = -1.0;
    public static double INTAKE_OUT_POWER = 1.0;
    public static double INTAKE_STOP_POWER = 0.0;

    public static double INTAKE_VEX_IN_POWER = -0.9;
    public static double INTAKE_VEX_OUT_POWER = 0.9;
    public static double INTAKE_VEX_STOP_POWER = 0.0;

    public static double STACK_ARM_OUT_POSITION = 0.64;
    public static double STACK_ARM_IN_POSITION = 0.12;

    public enum State {
        OFF,
        IN,
        OUT
    }

    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private CRServo vexMotor;
    private Servo stackArm;
    private HardwareMap hardwareMap;
    private State state = State.OFF;
    private boolean isStackArmOut;

    private Intake() {}

    public Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        this.rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        this.vexMotor = hardwareMap.get(CRServo.class, "vexMotor");
        this.stackArm = hardwareMap.get(Servo.class, "stackArm");
        this.isStackArmOut = false;
        this.moveStackArmIn();
        this.stop();
    }

    public State getState() {
        return state;
    }

    public void startIn() {
        leftIntake.setPower(ROLLER_IN_POWER);
        rightIntake.setPower(INTAKE_IN_POWER);
        vexMotor.setPower(INTAKE_VEX_IN_POWER);
        state = State.IN;
    }

    public void startOut() {
        leftIntake.setPower(ROLLER_OUT_POWER);
        rightIntake.setPower(INTAKE_OUT_POWER);
        vexMotor.setPower(INTAKE_VEX_OUT_POWER);
        state = State.OUT;
    }

    public void stop() {
        leftIntake.setPower(INTAKE_STOP_POWER);
        rightIntake.setPower(INTAKE_STOP_POWER);
        vexMotor.setPower(INTAKE_VEX_STOP_POWER);
        state = State.OFF;
    }

    public void moveStackArmOut() {
        isStackArmOut = true;
        stackArm.setPosition(STACK_ARM_OUT_POSITION);
    }

    public void moveStackArmIn() {
        isStackArmOut = false;
        stackArm.setPosition(STACK_ARM_IN_POSITION);
    }

    public boolean isStackArmOut() {
        return isStackArmOut;
    }
}
