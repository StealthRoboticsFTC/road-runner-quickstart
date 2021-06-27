package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class Shooter {
    public enum State {
        OFF,
        RAMP_UP,
        RUNNING,
        FIRING
    }

    public static double MAX_VELOCITY = 2500;
    public static double TARGET_VELOCITY = MAX_VELOCITY * 0.58;

    public static double TARGET_POWERSHOT_VELOCITY = MAX_VELOCITY * 0.5;

    public static PIDCoefficients VELOCITY_PID = new PIDCoefficients(22, 0, 0);
    public static double kS = 0.1 * MAX_VELOCITY;
    public static double kV = 1.0 - (kS / MAX_VELOCITY);

    public static double SHOOTER_STOP_POWER = 0.0;
//    public static double MAX_ACCEL = 1700;
    public static double MAX_ACCEL = 1500;
    public static double RAMP_UP_FINALIZE_TIME = 0.4;

    public static double OUT_ARM_POSITION = 0.82;
    public static double IN_ARM_POSITION = 0.69;
    public static double ARM_OUT_TIME = 0.1;
    public static double ARM_IN_TIME = 0.24;

    public static Vector2d GOAL_POSITION = new Vector2d(124, 106);

    private DcMotorEx frontShooter;
    private DcMotorEx backShooter;
    private Servo shooterArm;
    private Servo shooterFlap;

    private Intake intake;

    private PIDFController velocityController;

    private State state;

    private ElapsedTime armWaitTime = new ElapsedTime();
    private ElapsedTime rampTime = new ElapsedTime();

    private double startVelocity = 0;
    private double targetVelocity = TARGET_VELOCITY;
    private int shotsRemaining = 0;
    private boolean armIsIn = true;

    private Shooter() {}

    public Shooter(HardwareMap hardwareMap, Intake intake) {
        this.frontShooter = hardwareMap.get(DcMotorEx.class, "frontShooter");
        this.backShooter = hardwareMap.get(DcMotorEx.class, "backShooter");
        this.shooterArm = hardwareMap.get(Servo.class, "shooterArm");
        this.shooterFlap = hardwareMap.get(Servo.class, "shooterFlap");
        this.intake = intake;
        this.velocityController = new PIDFController(VELOCITY_PID, kV, 0.0, kS);
        this.stop();
        this.moveArmIn();
    }

    public void update() {
        if (state != State.OFF) {
            double power = velocityController.update(getVelocity()) / MAX_VELOCITY;
            setPower(power);
        }

        switch (state) {
            case OFF:
                break;
            case RAMP_UP:
                double dv = targetVelocity - startVelocity;
                double baseTime = Math.abs(dv) / MAX_ACCEL;
                double t = Math.min(Math.abs(rampTime.seconds() / baseTime), 1);
                setVelocity(dv * t + startVelocity);
                if (rampTime.seconds() > baseTime + RAMP_UP_FINALIZE_TIME) {
                    state = State.RUNNING;
                    setVelocity(targetVelocity);
                }
                break;
            case FIRING:
                if (armIsIn && armWaitTime.seconds() >= ARM_IN_TIME) {
                    moveArmOut();
                    armWaitTime.reset();
                }
                if (!armIsIn && armWaitTime.seconds() >= ARM_OUT_TIME) {
                    moveArmIn();
                    shotsRemaining--;
                    armWaitTime.reset();
                }

                if (shotsRemaining == 0) {
                    state = State.RUNNING;
                    moveArmIn();
                }
            case RUNNING:
                break;
        }
    }

    public State getState() {
        return state;
    }

    // TODO: revert publics?
    public void setVelocity(double velocity) {
        double vel = Math.round(Math.min(velocity, targetVelocity) / 20.0) * 20.0;
        velocityController.setTargetPosition(vel);
        velocityController.setTargetVelocity(vel);
    }

    public double getVelocity() {
        return frontShooter.getVelocity();
    }

    public void setPower(double power) {
        frontShooter.setPower(power);
        backShooter.setPower(power);
    }

    public void startRampUp() {
        rampTime.reset();
        targetVelocity = TARGET_VELOCITY;
        startVelocity = getVelocity();
        state = State.RAMP_UP;
    }

    public void startPowershotRampUp() {
        rampTime.reset();
        targetVelocity = TARGET_POWERSHOT_VELOCITY;
        startVelocity = getVelocity();
        state = State.RAMP_UP;
    }

    public void stop() {
        setPower(SHOOTER_STOP_POWER);
        moveArmIn();
        if (!intake.isStackArmOut()) {
            intake.moveStackArmIn();
        }
        state = State.OFF;
    }

    public void fire(int shotsToFire) {
        if (state == State.RUNNING) {
            if (!intake.isStackArmOut()) {
                intake.moveStackArmShoot();
            }
            moveArmOut();
            shotsRemaining = shotsToFire;
            armWaitTime.reset();
            state = State.FIRING;
        }
    }

    private void moveArmIn() {
        shooterArm.setPosition(IN_ARM_POSITION);
        armIsIn = true;
    }

    private void moveArmOut() {
        shooterArm.setPosition(OUT_ARM_POSITION);
        armIsIn = false;
    }

    public void setFlapPosition(double position) {
        shooterFlap.setPosition(position);
    }
}
