package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleArm {

    public static double ARM_UP_POSITION = 0.5;
    public static double ARM_DOWN_POSITION= 0.0;
    public static double GRIP_OPEN_POSITION=0.5;
    public static double GRIP_CLOSE_POSITION=0.0;

    protected Servo arm;
    protected Servo pincher;
    private HardwareMap hardwareMap;
    private boolean armDown;
    private boolean gripOpen;

    private WobbleArm(){}

    public WobbleArm(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.arm = hardwareMap.get(Servo.class, "arm");
        this.pincher = hardwareMap.get(Servo.class, "pincher");
        this.gripOpen();
        this.moveDown();

    }
    public void moveUp() {
        arm.setPosition(ARM_UP_POSITION);
        armDown=false;
    }

    public void moveDown() {
        arm.setPosition(ARM_DOWN_POSITION);
        armDown=true;
    }

    public void gripOpen() {
        pincher.setPosition(GRIP_OPEN_POSITION);
        gripOpen = true;
    }

    public void gripClose() {
        pincher.setPosition(GRIP_CLOSE_POSITION);
        gripOpen = false;

    }

    public boolean isArmDown() {
        return armDown;
    }
    public boolean isGripOpen () {
        return gripOpen;
    }
}
