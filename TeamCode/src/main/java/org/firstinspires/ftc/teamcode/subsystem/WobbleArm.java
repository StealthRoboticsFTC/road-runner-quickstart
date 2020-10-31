package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleArm {

    protected Servo arm;
    protected Servo pincher;
    private HardwareMap hardwareMap;

    private WobbleArm(){}

    public WobbleArm(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.arm = hardwareMap.get(Servo.class, "arm");
        this.pincher = hardwareMap.get(Servo.class, "pincher");
    }


    public void moveUp() {
        arm.setPosition(0.5);
    }

    public void moveDown() {
        arm.setPosition(0.0);
    }

    public void gripOpen() {
        pincher.setPosition(0.5);
    }

    public void gripClose() {
        pincher.setPosition(0.0);

    }
}
