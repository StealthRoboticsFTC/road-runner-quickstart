package org.firstinspires.ftc.teamcode.main

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.acmerobotics.roadrunner.util.EPSILON
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL
import org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystem.Shooter
import org.firstinspires.ftc.teamcode.subsystem.WobbleArm
import org.firstinspires.ftc.teamcode.vision.UGContourRingDetector
import org.firstinspires.ftc.teamcode.vision.UGContourRingPipeline
import java.lang.Math.toRadians
import kotlin.collections.ArrayList

@Autonomous
class AutoKt: LinearOpMode() {
    private val velConstraint = MinVelocityConstraint(java.util.Arrays.asList(
        AngularVelocityConstraint(toRadians(90.0)),
        MecanumVelocityConstraint(5.0, TRACK_WIDTH)
    ))
    private val accelConstraint = ProfileAccelerationConstraint(MAX_ACCEL)

    private val startPose = Pose2d(-63.0, 19.0, 0.0)

    private lateinit var drive: SampleMecanumDrive
    private lateinit var arm: WobbleArm
    private lateinit var shooter: Shooter

    private fun midTrajectories(startPose: Pose2d): List<Trajectory> {
        val list = ArrayList<Trajectory>()

        val builder2 = drive.trajectoryBuilder(startPose, true)
        builder2.splineTo(Vector2d(-4.0, 40.0), 180.0.toRadians)
        list.add(builder2.build())

        val builder3 = drive.trajectoryBuilder(list[list.size - 1].end(), 90.0.toRadians)
        builder3
                .splineToSplineHeading(Pose2d(-28.0, 51.0, 180.0.toRadians), 180.0.toRadians)
                .addTemporalMarker(1.0, -2.5) { arm.moveToPickup() }
                .splineTo(Vector2d(-36.0, 51.0), 180.0.toRadians, velConstraint, accelConstraint)
        list.add(builder3.build())

        return list
    }

    private fun zeroTrajectories(): List<Trajectory> {
        val list = ArrayList<Trajectory>()

        val builder1 = drive.trajectoryBuilder(startPose, startPose.heading)
        builder1.splineTo(Vector2d(22.0, 45.0), 90.0.toRadians)
        list.add(builder1.build())

        val midTrajectories = midTrajectories(list[list.size - 1].end())
        for (trajectory in midTrajectories) list.add(trajectory)

        val builder4 = drive.trajectoryBuilder(list[list.size - 1].end(), 0.0.toRadians)
        builder4.splineToSplineHeading(Pose2d(-20.0, 52.0, 0.0.toRadians), 0.0.toRadians - EPSILON)
        builder4.splineTo(Vector2d(-9.0, 52.0), 0.0.toRadians)
        list.add(builder4.build())

        val builder5 = drive.trajectoryBuilder(list[list.size - 1].end(), 270.0.toRadians)
        builder5.splineToConstantHeading(Vector2d(8.0, 36.0), 0.0.toRadians)
        list.add(builder5.build())

        return list
    }

    private fun oneTrajectories(): List<Trajectory> {
        val list = ArrayList<Trajectory>()

        val builder1 = drive.trajectoryBuilder(startPose, startPose.heading)
        builder1.splineTo(Vector2d(20.0, 36.0), 0.0.toRadians)
        list.add(builder1.build())

        val midTrajectories = midTrajectories(list[list.size - 1].end())
        for (trajectory in midTrajectories) list.add(trajectory)

        val builder4 = drive.trajectoryBuilder(list[list.size - 1].end(), 0.0.toRadians)
        builder4.splineToSplineHeading(Pose2d(-26.0, 51.0, 0.0.toRadians), 0.0.toRadians - EPSILON)
        builder4.splineTo(Vector2d(14.0, 31.0), 0.0.toRadians)
        list.add(builder4.build())

        val builder5 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
        builder5.back(10.0)
        list.add(builder5.build())

        return list
    }

    private fun fourTrajectories(): List<Trajectory> {
        val list = ArrayList<Trajectory>()

        val builder1 = drive.trajectoryBuilder(startPose, startPose.heading)
        builder1.splineTo(Vector2d(54.0, 42.0), 60.0.toRadians)
        list.add(builder1.build())

        val midTrajectories = midTrajectories(list[list.size - 1].end())
        for (trajectory in midTrajectories) list.add(trajectory)

        val builder4 = drive.trajectoryBuilder(list[list.size - 1].end(), 0.0.toRadians)
        builder4.splineToSplineHeading(Pose2d(-20.0, 56.0, 0.0.toRadians), 0.0.toRadians - EPSILON)
        builder4.splineTo(Vector2d(42.0, 56.0), 0.0.toRadians)
        list.add(builder4.build())

        val builder5 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
        builder5.splineTo(Vector2d(8.0, 36.0), 180.0.toRadians)
        list.add(builder5.build())

        return list
    }

    private fun update() {
        drive.update()
        shooter.update()
    }

    private fun sleepUpdate(ms: Long) {
        val elapsedTime = ElapsedTime()
        while (elapsedTime.milliseconds() < ms && !isStopRequested) {
            update()
        }
    }

    override fun runOpMode() {
        drive = SampleMecanumDrive(hardwareMap)
        arm = WobbleArm(hardwareMap)
        shooter = Shooter(hardwareMap, drive)
        arm.gripClose()

        val detector = UGContourRingDetector(hardwareMap, "webcam", telemetry, true)

        detector.init()

        val zeroTrajectories = zeroTrajectories()
        val oneTrajectories = oneTrajectories()
        val fourTrajectories = fourTrajectories()

        waitForStart()
        drive.poseEstimate = startPose

        val list = when (detector.height) {
            UGContourRingPipeline.Height.ZERO -> zeroTrajectories
            UGContourRingPipeline.Height.ONE -> oneTrajectories
            UGContourRingPipeline.Height.FOUR -> fourTrajectories
        }
//        val list = fourTrajectories
        detector.camera.closeCameraDeviceAsync {  }

        drive.followTrajectory(list[0])
        arm.moveToDropOff()
        sleepUpdate(1000)
        arm.gripOpen()
        sleepUpdate(500)
        arm.moveToCarry()

        drive.followTrajectory(list[1])
        shooter.startRampUp()
        while (shooter.state == Shooter.State.RAMP_UP && !isStopRequested) {
            update()
        }
        shooter.fire(3)
        while (shooter.state == Shooter.State.FIRING && !isStopRequested) {
            update()
        }
        sleepUpdate(1500)

        shooter.stop()

        drive.followTrajectory(list[2])
        arm.gripClose()
        sleepUpdate(300)
        arm.moveToCarry()
        sleepUpdate(1000)

        drive.followTrajectory(list[3])
        arm.moveToDropOff()
        sleepUpdate(1000)
        arm.gripOpen()
        sleepUpdate(500)

        drive.followTrajectory(list[4])
        arm.moveToInitial()
        PoseStorage.pose = drive.poseEstimate

//        val timer = ElapsedTime()
//        while (!isStopRequested) {
//            if (timer.seconds() > .7) {
//                if (arm.armPosition == WobbleArm.ArmPosition.CARRY) {
//                    arm.moveToDropOff()
//                } else {
//                    arm.moveToCarry()
//                }
//            }
//        }
    }
}

val Double.toRadians get() = (Math.toRadians(this))
