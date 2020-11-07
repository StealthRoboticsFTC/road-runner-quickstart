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

        boolean isADown=false;
        boolean isXDown=false;

        while (!isStopRequested()) {
            if (gamepad2.a && !isADown) {

                //Initial -> carry
                //Carry -> dropoff
                //Drop-off -> pickup
                //Pickup -> carry
                switch(wobbleArm.getArmPosition()){
                    case INITIAL:
                    case PICKUP:
                        wobbleArm.moveToCarry();
                        break;
                    case CARRY:
                        wobbleArm.moveToDropOff();
                        break;
                    case DROPOFF:
                        wobbleArm.moveToPickup();
                        break;

                }
            }
            if (gamepad2.x && !isXDown) {
                if (wobbleArm.isGripOpen()){
                    wobbleArm.gripClose();
                }else {
                    wobbleArm.gripOpen();
                }
            }
            isADown=gamepad2.a;
            isXDown=gamepad2.x;
        }
    }
}
