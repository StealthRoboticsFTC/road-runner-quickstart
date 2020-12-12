package org.firstinspires.ftc.teamcode.subsystem;

import android.graphics.drawable.GradientDrawable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
When a certain set of buttons are pressed, the robot will move from anywhere on the field to
a preset series of points on the field starting with a point where it will shoot one ring and knock
down the left power-shot goal, then straife to the right where it will shoot another ring and
knock down the middle power-shot goal, then straife to the right again where it will knock down the
power-shot goal, and then give control back to the drivers.

1. Envoke roadrunner code to move to left most point
2. Start up the shooter motors
3. Shoot one ring
4. Using roadrunner, straife to the middle point
5. Shoot one ring
6. Using roadrunner, straife to the right point
7. Shoot one ring
8. Turn off shooter motors
 */

/*
left: (-10.3522, 23.366)
middle: (-10.3522, 16.8147)
right: (-11.122, 7.264)
 */

/*
Line to add a trajectory
drive.trajectoryBuilder(drive.getPoseEstimate()).lineToLinearHeading(pose1).build()
 */

public class AutonomousPowerShot {
    public enum State {
        OFF,
        MOVING,
        ADJUSTING_IMU,
        ADJUSTING_DISTANCE,
        SHOOTING,
        WAITING
    }

    private double SHOOTING_SECONDS = 0.5;

    private Pose2d ZERO_POSE = new Pose2d(-10.3522, 23.366);
    private Pose2d ONE_POSE = new Pose2d(-10.3522, 16.8147);
    private Pose2d TWO_POSE = new Pose2d(-11.2122, 7.264);

    private State state = State.OFF;

    private ElapsedTime timer = new ElapsedTime();

    private Shooter shooter;
    private SampleMecanumDrive drive;
    private DistanceSensor leftSensor;
    private DistanceSensor rightSensor;
    private BNO055IMU imu;

    private int shotNumber = 0;

    public AutonomousPowerShot(Shooter shooter, SampleMecanumDrive drive, DistanceSensor leftSensor,
                               DistanceSensor rightSensor, BNO055IMU imu) {
        this.shooter = shooter;
        this.drive = drive;
        this.leftSensor = leftSensor;
        this.rightSensor = rightSensor;
        this.imu = imu;
    }

    public void update() {
        System.out.println("**********state = " + state);
        switch (state) {
            case OFF:
                break;
            case MOVING:
                if(! drive.isBusy()) {
                    state = State.ADJUSTING_IMU;
//                    shooter.fire(1);
                }
                break;
            case SHOOTING:
                if(shooter.getState() != Shooter.State.FIRING) {
                    state = State.WAITING;
                    timer.reset();
                }
                break;
            case ADJUSTING_IMU:
                if(! drive.isBusy()) {
                    Orientation angularOrientation = imu.getAngularOrientation();
                    System.out.println("************Angular orientation = " + angularOrientation.firstAngle);
                    if (angularOrientation.firstAngle <= 1.5 || angularOrientation.firstAngle >= 358.5) {
                        state = State.ADJUSTING_DISTANCE;
                    } else if (angularOrientation.firstAngle > 0 && angularOrientation.firstAngle <= 180) {
                        drive.turn(Math.toRadians(-1 * angularOrientation.firstAngle));
                        System.out.println("************turning left");
                    } else if (angularOrientation.firstAngle > 180) {
                        drive.turn(Math.toRadians( - angularOrientation.firstAngle));
                        System.out.println("***********turning right");
                    }
                }
                break;
            case ADJUSTING_DISTANCE:
                if(! drive.isBusy()) {
                    double difference = leftSensor.getDistance(DistanceUnit.INCH) - rightSensor.getDistance(DistanceUnit.INCH);
                    System.out.println("difference = " + difference)
                    if (Math.abs(difference) < 0.25) {
                        state = State.SHOOTING;
                        shooter.fire(1);
                    } else if (difference > 0) {
                        drive.turn(Math.toRadians(1));
                    } else if (difference < 0) {
                        drive.turn(Math.toRadians(-0.1));
                    }
                }
                break;
            case WAITING:
                if(timer.seconds() > SHOOTING_SECONDS) {
                    shotNumber++;
                    if(shotNumber > 2) {
                        state = State.OFF;
                    } else {
                        followTrajectory();
                    }
                }
                break;
        }
    }

    public void start() {
        state = State.MOVING;
        shotNumber = 0;
        followTrajectory();
    }

    public void stop() {
        state = State.OFF;
        drive.stopFollowing();
    }

    private void followTrajectory() {
        Pose2d targetPose;
        targetPose = ZERO_POSE;
//        switch (shotNumber) {
//            case 1:
//                targetPose = ONE_POSE;
//                break;
//            case 2:
//                targetPose = TWO_POSE;
//                break;
//            default:
//                targetPose = ZERO_POSE;
//                break;
//        }

        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(targetPose)
                .build();

        drive.followTrajectoryAsync(trajectory);

    }

    public State getState() {
        return state;
    }

}