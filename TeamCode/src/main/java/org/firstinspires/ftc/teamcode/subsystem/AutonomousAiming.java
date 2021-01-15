package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
    private ElapsedTime pidTimer = new ElapsedTime();

    private int currentPowershot = 0;
    private double lastX = Double.POSITIVE_INFINITY;

    private final double PID_TIME_TOLERANCE = 0.1;

    private final double TARGET = 320/2;
    private final double AIMING_TOLERANCE = 10;

    private final double POWERSHOT_WAIT_TIME = 0.4;

    private AimingPipeline aiming = new AimingPipeline();

    public PIDCoefficients coefficients = new PIDCoefficients(0.1, 0.05, 0);
    private PIDFController pidControl = new PIDFController(coefficients);

    public AutonomousAiming(HardwareMap hardwareMap, SampleMecanumDrive drive, Shooter shooter) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new AimingPipeline());
        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));

        this.drive = drive;
        this.shooter = shooter;
    }

    public void update() {
        double currentX = 0;

        switch (state) {
            case OFF:
                break;
            case AIMING_HIGH_GOAL:
                currentX = aiming.getGoalCenterX();
                if(Math.abs(currentX - TARGET) < AIMING_TOLERANCE && shooter.getState() == Shooter.State.RUNNING) {
                    shooter.fire(3);
                    drive.setDriveSignal(new DriveSignal());
                    pidControl.reset();
                    state = State.SHOOTING_HIGH_GOAL;
                }
                break;
            case AIMING_POWERSHOT:
                currentX = aiming.getPowershotsCenterX()[currentPowershot];
                if(Math.abs(currentX - TARGET) < AIMING_TOLERANCE && shooter.getState() == Shooter.State.RUNNING) {
                    shooter.fire(1);
                    drive.setDriveSignal(new DriveSignal());
                    pidControl.reset();
                    state = State.SHOOTING_POWERSHOT;
                }
                break;
            case SHOOTING_HIGH_GOAL:
                if (shooter.getState() != Shooter.State.FIRING) {
                    state = State.OFF;
                }
                break;
            case SHOOTING_POWERSHOT:
                if (shooter.getState() != Shooter.State.FIRING) {
                    currentPowershot++;
                    if (currentPowershot <= 2) {
                        state = State.WAITING_POWERSHOT;
                        timer.reset();
                    } else {
                        state = State.OFF;
                    }
                }
                currentPowershot++;
                break;
            case WAITING_POWERSHOT:
                if (timer.seconds() > POWERSHOT_WAIT_TIME) {
                    state = State.AIMING_POWERSHOT;
                }
                break;
        }
        if((state == State.AIMING_HIGH_GOAL || state == State.AIMING_POWERSHOT) && (currentX != lastX || pidTimer.seconds() > PID_TIME_TOLERANCE)) {
            lastX = currentX;
            pidControl.setTargetPosition(TARGET);
            pidTimer.reset();
            double output = pidControl.update(currentX);
            DriveSignal driveSignal = new DriveSignal(new Pose2d(0, 0, output));
            drive.setDriveSignal(driveSignal);
        }
    }

    public void startHighGoal() {
        state = State.AIMING_HIGH_GOAL;
        shooter.startRampUp();
    }

    public void startPowershot() {
        state = State.AIMING_POWERSHOT;
        shooter.startPowershotRampUp();
        currentPowershot = 0;
    }

    public void stop() {
        state = State.OFF;
        pidControl.reset();
    }

    public State getState() {
        return state;
    }

}
