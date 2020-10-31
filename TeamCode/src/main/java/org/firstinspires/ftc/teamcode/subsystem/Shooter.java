package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    private HardwareMap hardwareMap;
    private DcMotor frontShooter;
    private DcMotor backShooter;
    private Servo shooterArm;

    enum State {
        OFF,
        RAMP_UP,
        RUNNING,
        FIRING
    }

    private State shooterState;

    private static double SHOOTER_STOP_POWER = 0.0;
    private static double MAX_POWER = 1.0;
    private static double RAMP_UP_TIME = 2.5;

    private static double MAX_ARM_POSITION = 0.5;
    private static double MIN_ARM_POSITION = 0.0;
    private static double ARM_OUT_TIME = 0.1;
    private static double ARM_IN_TIME = 0.05;

    private int shotsRemaining = 0;
    private boolean armIsIn = true;
    private ElapsedTime armWaitTime = new ElapsedTime();

    private ElapsedTime rampTime = new ElapsedTime();

    public Shooter(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.frontShooter = hardwareMap.get(DcMotor.class, "frontShooter");
        this.backShooter = hardwareMap.get(DcMotor.class, "backShooter");
        this.shooterArm = hardwareMap.get(Servo.class, "shooterArm");
        this.stop();
        this.moveArmIn();
    }

    public void update() {
        switch (shooterState) {
            case OFF:
                break;
            case RAMP_UP:
                double motorPower = (MAX_POWER / RAMP_UP_TIME) * rampTime.seconds();
                setPower(motorPower);
                if (rampTime.seconds() > RAMP_UP_TIME) {
                    shooterState = State.RUNNING;
                }
                break;
            case RUNNING:
                break;
            case FIRING:
                if (armIsIn && armWaitTime.seconds() >= ARM_IN_TIME) {
                    moveArmOut();
                    shotsRemaining--;
                    armWaitTime.reset();
                } else if (armWaitTime.seconds() >= ARM_OUT_TIME) {
                    moveArmIn();
                    armWaitTime.reset();
                }

                if (shotsRemaining == 0) {
                    shooterState = State.RUNNING;
                }
                break;
        }
    }

    private void setPower(double power) {
        double clipPower = Math.min(power, MAX_POWER);
        frontShooter.setPower(clipPower);
        backShooter.setPower(clipPower);
    }

    public void startRampUp() {
        rampTime.reset();
        shooterState = State.RAMP_UP;
    }

    public void stop() {
        frontShooter.setPower(SHOOTER_STOP_POWER);
        backShooter.setPower(SHOOTER_STOP_POWER);
        shooterState = State.OFF;
    }

    public void fire() {
        if (shooterState == State.RUNNING) {
            shooterState = State.FIRING;
            shotsRemaining = 3;
            armWaitTime.reset();
        }
    }

    private void moveArmIn() {
        shooterArm.setPosition(MIN_ARM_POSITION);
        armIsIn = true;
    }

    private void moveArmOut() {
        shooterArm.setPosition(MAX_ARM_POSITION);
        armIsIn = false;
    }
}
