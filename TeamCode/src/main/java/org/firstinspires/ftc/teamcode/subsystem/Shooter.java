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

    private State shooterState;

    public static PIDCoefficients VELOCITY_PID = new PIDCoefficients(10, 0, 0.3);
    public static double kV = 0.0;
    public static double kS = 0.0;

    public static double MAX_VELOCITY = 1.0;
    public static double TARGET_VELOCITY = MAX_VELOCITY * 0.8;

    public static double SHOOTER_STOP_POWER = 0.0;
    public static double RAMP_UP_TIME = 1.0;

    public static double OUT_ARM_POSITION = 0.0;
    public static double IN_ARM_POSITION = 0.12;
    public static double ARM_OUT_TIME = 0.1;
    public static double ARM_IN_TIME = 0.9;

    public static double CONVEYOR_MOVING_POWER = 0.5;
    public static double CONVEYOR_STOP_POWER = 0.0;

    public static Vector2d GOAL_POSITION = new Vector2d(124, 106);

    private HardwareMap hardwareMap;

    private DcMotorEx frontShooter;
    private DcMotorEx backShooter;
    private Servo shooterArm;
    private Servo shooterFlap;
    private CRServo conveyor;

    private SampleMecanumDrive drive;

    private PIDFController velocityController;

    private ElapsedTime armWaitTime = new ElapsedTime();
    private ElapsedTime rampTime = new ElapsedTime();

    private int shotsRemaining = 0;
    private boolean armIsIn = true;

    public Shooter(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        this.hardwareMap = hardwareMap;
        this.frontShooter = hardwareMap.get(DcMotorEx.class, "frontShooter");
        this.backShooter = hardwareMap.get(DcMotorEx.class, "backShooter");
        this.shooterArm = hardwareMap.get(Servo.class, "shooterArm");
        this.shooterFlap = hardwareMap.get(Servo.class, "shooterFlap");
        this.conveyor = hardwareMap.get(CRServo.class, "conveyor");
        this.drive = drive;
        this.velocityController = new PIDFController(VELOCITY_PID, kV, 0.0, kS);
        this.stop();
        this.moveArmIn();
    }

    public void update() {
        switch (shooterState) {
            case OFF:
                break;
            case RAMP_UP:
                double motorPower = (TARGET_VELOCITY / RAMP_UP_TIME) * rampTime.seconds();
                setPower(motorPower);
                if (rampTime.seconds() > RAMP_UP_TIME) {
                    shooterState = State.RUNNING;
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
                    shooterState = State.RUNNING;
                    moveArmIn();
                }
            case RUNNING:
                updateFlap();

                double power = velocityController.update(getVelocity()) / MAX_VELOCITY;
                setPower(power);
                break;
        }
    }

    public State getShooterState() {
        return shooterState;
    }

    // TODO: revert publics?
    public void setVelocity(double velocity) {
        velocityController.setTargetPosition(velocity);
        velocityController.setTargetVelocity(velocity);
    }

    public double getVelocity() {
        return backShooter.getVelocity();
    }

    public void setPower(double power) {
        frontShooter.setPower(power);
        backShooter.setPower(power);
    }

    public void startRampUp() {
        rampTime.reset();
        conveyor.setPower(CONVEYOR_MOVING_POWER);
        shooterState = State.RAMP_UP;
    }

    public void stop() {
        setPower(SHOOTER_STOP_POWER);
        conveyor.setPower(CONVEYOR_STOP_POWER);
        moveArmIn();
        shooterState = State.OFF;
    }

    public void fire() {
        if (shooterState == State.RUNNING) {
            moveArmIn();
            shotsRemaining = 3;
            armWaitTime.reset();
            shooterState = State.FIRING;
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

    private void updateFlap() {
//        double distance = drive.getPoseEstimate().vec().minus(GOAL_POSITION).norm();
//        double flapPosition = flapPositionTable.get(distance);
//        shooterFlap.setPosition(flapPosition);
    }

}
