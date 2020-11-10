package org.firstinspires.ftc.teamcode.parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.WobbleArm;

@Config

@TeleOp
public class WobbleArmTest extends LinearOpMode {

    private WobbleArm wobbleArm;
    public static int AP = -1;

    @Override
    public void runOpMode() {

        wobbleArm = new WobbleArm(hardwareMap);

        waitForStart();

        boolean isADown=false;
        boolean isXDown=false;

        while (!isStopRequested()) {
            if (gamepad1.a && !isADown) {

                AP = wobbleArm.getArmPosition().ordinal();
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
            if (gamepad1.x && !isXDown) {
                if (wobbleArm.isGripOpen()){
                    wobbleArm.gripClose();
                }else {
                    wobbleArm.gripOpen();
                }
            }
            isADown=gamepad1.a;
            isXDown=gamepad1.x;
        }
    }
}
