package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.AimingPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class AutonomousAiming {
    public enum State {
        OFF,
        AIMING_HIGH_GOAL,
        AIMING_POWERSHOT,
        SHOOTING_HIGH_GOAL,
        SHOOTING_POWERSHOT,
        WAITING_POWERSHOT
    }

    private State state = State.OFF;

    private final OpenCvWebcam webcam;
    private SampleMecanumDrive drive;
    private Shooter shooter;

    private ElapsedTime timer = new ElapsedTime();

    private double goal;

    private AimingPipeline aiming = new AimingPipeline();

    public PIDCoefficients coefficients = new PIDCoefficients(0.1, 0.05, 0);
    private PIDFController pidControl = new PIDFController(coefficients);

    public AutonomousAiming(HardwareMap hardwareMap, SampleMecanumDrive drive, Shooter shooter) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new AimingPipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            { webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); }
        });

        this.drive = drive;
        this.shooter = shooter;
    }

    public void update() {
        double currentX = 0;
        double currentY = 0;

        if(state)

        switch (state) {
            case OFF:
                break;
            case AIMING_HIGH_GOAL:
                goal = aiming.getGoalCenterX();

                break;
            case AIMING_POWERSHOT:
                if (!drive.isBusy() && shooter.getState() == Shooter.State.RUNNING) {
                    //state = State.SHOOTING;
                    shooter.fire(1);
                }
                break;
            case SHOOTING_HIGH_GOAL:
                if (shooter.getState() != Shooter.State.FIRING) {
                    state = State.WAITING_POWERSHOT;
                    timer.reset();
                }
                break;
            case SHOOTING_POWERSHOT:
                break;
            case WAITING_POWERSHOT:
                break;
        }
    }

    public void startHighGoal() {
        state = State.AIMING_HIGH_GOAL;
    }

    public void startPowershot() {
        state = State.AIMING_POWERSHOT
    }

}
