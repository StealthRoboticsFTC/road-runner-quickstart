package org.firstinspires.ftc.teamcode.parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.WobbleArm;

@Config

@TeleOp
public class WobbleArmTest extends LinearOpMode {

    private WobbleArm wobbleArm;

    @Override
    public void runOpMode() {

        wobbleArm = new WobbleArm(hardwareMap);

        waitForStart();

        boolean isXButtonDown=false;
        boolean isAButtonDown=false;

        while (!isStopRequested()) {
            if (gamepad2.x && !isXButtonDown) {
                switch(wobbleArm.getArmPosition()){
                    case INITIAL:
                    case PICKUP:
                    case DROPOFF:
                        wobbleArm.moveToCarry();
                        break;
                    case CARRY:
                        if (wobbleArm.isGripOpen()){
                            wobbleArm.moveToPickup();
                        }else {
                            wobbleArm.moveToDropOff();
                        }
                        break;
                }
            }

            if (gamepad2.a && !isAButtonDown) {
                if (!wobbleArm.isGripOpen()) {
                    wobbleArm.gripClose();
                } else {
                    wobbleArm.gripOpen();
                }
            }
            isXButtonDown=gamepad2.x;
            isAButtonDown=gamepad2.a;
        }
    }
}
