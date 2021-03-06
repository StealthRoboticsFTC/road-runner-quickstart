 package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.ModifiedPIDFController;
import org.firstinspires.ftc.teamcode.vision.AimingPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

 @Config
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
    private double lastX = Double.MAX_VALUE;
    private MovingStatistics lastXs = new MovingStatistics(5);
    private double lastDelta = 0.0;

    public static double PID_TIME_TOLERANCE = 0.2;

    public static double GAIN = 1.0;
    public static double EXPOSURE = 0.12;

    public static double TARGET = (320/2) - 10;
    public static double AIMING_POWERSHOT_TOLERANCE = 6;
    public static double AIMING_GOAL_TOLERANCE = 12;
    public static double AIMING_DELTA_TOLERANCE = 4;
    public static double MAX_SPEED = 0.5;
    public static double MIN_SPEED = 0.05;

    public static double POWERSHOT_WAIT_TIME = 1.3;

    private AimingPipeline aiming = new AimingPipeline();

    public static PIDCoefficients coefficients = new PIDCoefficients(0.012, 0.001,
            0.0);
    private ModifiedPIDFController pidControl = new ModifiedPIDFController(coefficients, 20.0);

    public AutonomousAiming(HardwareMap hardwareMap, SampleMecanumDrive drive, Shooter shooter) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(aiming);
        webcam.openCameraDeviceAsync(() -> {
            FtcDashboard.getInstance().startCameraStream(webcam, 30);
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            GainControl gainControl = webcam.getGainControl();
            gainControl.setGain((int) Range.scale(GAIN, 0, 1, gainControl.getMinGain(), gainControl.getMaxGain()));
            ExposureControl exposureControl = webcam.getExposureControl();
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure((long) Range.scale(EXPOSURE, 0, 1,
                    exposureControl.getMinExposure(TimeUnit.NANOSECONDS),
                    exposureControl.getMaxExposure(TimeUnit.NANOSECONDS)), TimeUnit.NANOSECONDS);
        });

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
                if(Math.abs(currentX - TARGET) < AIMING_GOAL_TOLERANCE && shooter.getState() == Shooter.State.RUNNING) {
                    lastXs.clear();
                    shooter.fire(3);
                    drive.setDriveSignal(new DriveSignal());
                    pidControl.reset();
                    state = State.SHOOTING_HIGH_GOAL;
                }
                break;
            case AIMING_POWERSHOT:
                currentX = aiming.getPowershotsCenterX()[currentPowershot];
                if(Math.abs(currentX - TARGET) < AIMING_POWERSHOT_TOLERANCE
                        && shooter.getState() == Shooter.State.RUNNING
                        && Math.abs(lastDelta) <= AIMING_DELTA_TOLERANCE) {
                    lastXs.clear();
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
                break;
            case WAITING_POWERSHOT:
                if (timer.seconds() > POWERSHOT_WAIT_TIME) {
                    state = State.AIMING_POWERSHOT;
                }
                break;
        }

        if((state == State.AIMING_HIGH_GOAL || state == State.AIMING_POWERSHOT)
                && (currentX != lastX || pidTimer.seconds() > PID_TIME_TOLERANCE)) {
            lastXs.add(currentX);
            lastDelta = (currentX - lastXs.getMean()) / lastXs.getCount();
            lastX = currentX;
            pidControl.setTargetPosition(TARGET);
            pidControl.setOutputBounds(-MAX_SPEED, MAX_SPEED);
            pidTimer.reset();
            double output = pidControl.update(currentX);
            DriveSignal driveSignal = new DriveSignal(new Pose2d(0, 0,
                    output + Math.signum(output) * MIN_SPEED));
            drive.setDriveSignal(driveSignal);
        }
    }

    public void startHighGoal() {
        state = State.AIMING_HIGH_GOAL;
        shooter.startRampUp();
        lastXs.clear();
    }

    public void startPowershot() {
        state = State.AIMING_POWERSHOT;
        shooter.startPowershotRampUp();
        lastXs.clear();
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
