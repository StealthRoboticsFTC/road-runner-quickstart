package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
//    public static double INTAKE_IN_POWER = 0.66;
//    public static double INTAKE_OUT_POWER = -0.6;

    public static double INTAKE_IN_POWER = 1;
    public static double INTAKE_OUT_POWER = -1;

    public static double INTAKE_STOP_POWER = 0.0;

    public static double STACK_ARM_OUT_POSITION = 0.355;
    public static double STACK_ARM_SHOOT_POSITION = 0.77;
    public static double STACK_ARM_IN_POSITION = 0.87;

    public enum State {
        OFF,
        IN,
        OUT
    }

    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private Servo stackArm;
    private State state = State.OFF;
    private boolean isStackArmOut;

    private Intake() {}

    public Intake(HardwareMap hardwareMap) {
        this.leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        this.rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        this.rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        this.stackArm = hardwareMap.get(Servo.class, "stackArm");
        this.isStackArmOut = false;
        this.moveStackArmIn();
        this.stop();
    }

    public State getState() {
        return state;
    }

    public void startIn() {
        leftIntake.setPower(INTAKE_IN_POWER);
        rightIntake.setPower(INTAKE_IN_POWER);
        state = State.IN;
    }

    public void startOut() {
        leftIntake.setPower(INTAKE_OUT_POWER);
        rightIntake.setPower(INTAKE_OUT_POWER);
        state = State.OUT;
    }

    public void stop() {
        leftIntake.setPower(INTAKE_STOP_POWER);
        rightIntake.setPower(INTAKE_STOP_POWER);
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

    public void moveStackArmShoot() {
        isStackArmOut = false;
        stackArm.setPosition(STACK_ARM_SHOOT_POSITION);
    }

    public boolean isStackArmOut() {
        return isStackArmOut;
    }
}
