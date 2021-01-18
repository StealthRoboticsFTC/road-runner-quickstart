package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class WobbleArm {
    public enum ArmPosition {
        INITIAL,
        CARRY,
        DROPOFF,
        PICKUP
    }

    public static double ARM_INITIAL_POSITION = 0.34;
    public static double ARM_CARRY_POSITION = 0.5;
    public static double ARM_DROPOFF_POSITION = 0.6;
    public static double ARM_PICKUP_POSITION = 0.685;
    public static double GRIP_OPEN_POSITION=0.55;
    public static double GRIP_CLOSE_POSITION=0.175;

    private Servo arm;
    private Servo pincher;
    private HardwareMap hardwareMap;
    private ArmPosition armPosition;
    private boolean gripOpen;

    private WobbleArm() {}

    public WobbleArm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.arm = hardwareMap.get(Servo.class, "arm");
        this.pincher = hardwareMap.get(Servo.class, "pincher");
        this.gripOpen();
        this.moveToInitial();
    }

    public void moveToInitial() {
        arm.setPosition(ARM_INITIAL_POSITION);
        armPosition = ArmPosition.INITIAL;
    }

    public void moveToCarry() {
        arm.setPosition(ARM_CARRY_POSITION);
        armPosition = ArmPosition.CARRY;
    }

    public void moveToDropOff() {
        arm.setPosition(ARM_DROPOFF_POSITION);
        armPosition = ArmPosition.DROPOFF;
    }

    public void moveToPickup() {
        arm.setPosition(ARM_PICKUP_POSITION);
        armPosition = ArmPosition.PICKUP;
    }

    public void gripOpen() {
        pincher.setPosition(GRIP_OPEN_POSITION);
        gripOpen = true;
    }

    public void gripClose() {
        pincher.setPosition(GRIP_CLOSE_POSITION);
        gripOpen = false;

    }

    public ArmPosition getArmPosition() {
        return armPosition;
    }
    public boolean isGripOpen () {
        return gripOpen;
    }
}
