package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class Shooter {
    private HardwareMap hardwareMap;
    private DcMotor frontShooter;
    private DcMotor backShooter;
    private Servo shooterArm;
    private Servo shooterFlap;
    private CRServo conveyor;

    private SampleMecanumDrive drive;

//    private InterpLUT flapPositionTable = InterpLUT.createLUT(
//            Arrays.asList(0.0), Arrays.asList(0.0)
//    );

    public enum State {
        OFF,
        RAMP_UP,
        RUNNING,
        FIRING
    }

    private State shooterState;

    public static double SHOOTER_STOP_POWER = 0.0;
    public static double MAX_POWER = 1.0;
    public static double RAMP_UP_TIME = 1.0;

    public static double OUT_ARM_POSITION = 0.0;
    public static double IN_ARM_POSITION = 0.12;
    public static double ARM_OUT_TIME = 0.3;
    public static double ARM_IN_TIME = 0.9;

    public static double CONVEYOR_MOVING_POWER = 0.5;
    public static double CONVEYOR_STOP_POWER = 0.0;

    public static double BASELINE_VOLTAGE = 12.6;

    public static Vector2d GOAL_POSITION = new Vector2d(124, 106);

    private double initVoltage = 0.0;
    private int shotsRemaining = 0;
    private boolean armIsIn = true;
    private ElapsedTime armWaitTime = new ElapsedTime();
    private ElapsedTime rampTime = new ElapsedTime();

    public Shooter(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        this.hardwareMap = hardwareMap;
        this.frontShooter = hardwareMap.get(DcMotor.class, "frontShooter");
        this.backShooter = hardwareMap.get(DcMotor.class, "backShooter");
        this.shooterArm = hardwareMap.get(Servo.class, "shooterArm");
        this.shooterFlap = hardwareMap.get(Servo.class, "shooterFlap");
        this.conveyor = hardwareMap.get(CRServo.class, "conveyor");
        this.drive = drive;
        this.initVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
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
                updateFlap();
                break;
            case FIRING:
                updateFlap();
                System.err.println("****** " + armWaitTime.seconds() + " : " + armIsIn);
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
                break;
        }
    }

    public State getShooterState() {
        return shooterState;
    }

    private void setPower(double power) {
        double clipPower = Math.min(power, MAX_POWER) * BASELINE_VOLTAGE / initVoltage;
        frontShooter.setPower(clipPower);
        backShooter.setPower(clipPower);
    }

    public void startRampUp() {
        rampTime.reset();
        conveyor.setPower(CONVEYOR_MOVING_POWER);
        shooterState = State.RAMP_UP;
    }

    public void stop() {
        frontShooter.setPower(SHOOTER_STOP_POWER);
        backShooter.setPower(SHOOTER_STOP_POWER);
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
