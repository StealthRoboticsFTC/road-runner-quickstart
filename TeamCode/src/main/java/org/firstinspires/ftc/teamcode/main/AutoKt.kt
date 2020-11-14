package org.firstinspires.ftc.teamcode.main

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.drive.DriveConstants.BASE_CONSTRAINTS
import org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystem.Shooter
import org.firstinspires.ftc.teamcode.subsystem.WobbleArm
import org.firstinspires.ftc.teamcode.vision.UGContourRingDetector
import org.firstinspires.ftc.teamcode.vision.UGContourRingPipeline
import java.util.*
import kotlin.collections.ArrayList

@Autonomous
class AutoKt: LinearOpMode() {
    private val slowConstraints = DriveConstraints(5.0, BASE_CONSTRAINTS.maxAccel, BASE_CONSTRAINTS.maxJerk, BASE_CONSTRAINTS.maxAngVel, BASE_CONSTRAINTS.maxAngAccel, BASE_CONSTRAINTS.maxAngJerk)
    private val slowCombinedConstraints = MecanumConstraints(slowConstraints, TRACK_WIDTH)

    private val startPose = Pose2d(-60.0, 19.0, 0.0)

    private lateinit var drive: SampleMecanumDrive
    private lateinit var arm: WobbleArm
    private lateinit var shooter: Shooter

    private fun midTrajectories(startPose: Pose2d): List<Trajectory> {
        val list = ArrayList<Trajectory>()

        val builder2 = drive.trajectoryBuilder(startPose, true)
        builder2.splineTo(Vector2d(-20.0, 36.0), 180.0.toRadians)
        list.add(builder2.build())

        val builder3 = drive.trajectoryBuilder(list[list.size - 1].end(), 90.0.toRadians)
        builder3
                .splineToSplineHeading(Pose2d(-28.0, 57.0, 180.0.toRadians), 180.0.toRadians)
                .addTemporalMarker(0.0, -1.5) { arm.moveToPickup() }
                .splineTo(Vector2d(-36.0, 57.0), 180.0.toRadians, slowCombinedConstraints)
        list.add(builder3.build())

        return list
    }

    private fun zeroTrajectories(): List<Trajectory> {
        val list = ArrayList<Trajectory>()

        val builder1 = drive.trajectoryBuilder(startPose, startPose.heading)
        builder1.splineTo(Vector2d(22.0, 40.0), 90.0.toRadians)
        list.add(builder1.build())

        val midTrajectories = midTrajectories(list[list.size - 1].end())
        for (trajectory in midTrajectories) list.add(trajectory)

        val builder4 = drive.trajectoryBuilder(list[list.size - 1].end(), 0.0.toRadians)
        builder4.splineToSplineHeading(Pose2d(-20.0, 57.0, 0.0.toRadians), 0.0.toRadians)
        builder4.splineTo(Vector2d(-6.0, 57.0), 0.0.toRadians)
        list.add(builder4.build())

        val builder5 = drive.trajectoryBuilder(list[list.size - 1].end(), 270.0.toRadians)
        builder5.splineToConstantHeading(Vector2d(8.0, 36.0), 0.0.toRadians)
        list.add(builder5.build())

        return list
    }

    private fun oneTrajectories(): List<Trajectory> {
        val list = ArrayList<Trajectory>()

        val builder1 = drive.trajectoryBuilder(startPose, startPose.heading)
        builder1.splineTo(Vector2d(26.0, 26.0), 0.0.toRadians)
        list.add(builder1.build())

        val midTrajectories = midTrajectories(list[list.size - 1].end())
        for (trajectory in midTrajectories) list.add(trajectory)

        val builder4 = drive.trajectoryBuilder(list[list.size - 1].end(), 0.0.toRadians)
        builder4.splineToSplineHeading(Pose2d(-26.0, 57.0, 0.0.toRadians), 0.0.toRadians)
        builder4.splineTo(Vector2d(19.0, 26.0), 0.0.toRadians)
        list.add(builder4.build())

        val builder5 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
        builder5.back(10.0)
        list.add(builder5.build())

        return list
    }

    private fun fourTrajectories(): List<Trajectory> {
        val list = ArrayList<Trajectory>()

        val builder1 = drive.trajectoryBuilder(startPose, startPose.heading)
        builder1.splineTo(Vector2d(56.0, 40.0), 60.0.toRadians)
        list.add(builder1.build())

        val midTrajectories = midTrajectories(list[list.size - 1].end())
        for (trajectory in midTrajectories) list.add(trajectory)

        val builder4 = drive.trajectoryBuilder(list[list.size - 1].end(), 0.0.toRadians)
        builder4.splineToSplineHeading(Pose2d(-20.0, 57.0, 0.0.toRadians), 0.0.toRadians)
        builder4.splineTo(Vector2d(46.0, 57.0), 0.0.toRadians)
        list.add(builder4.build())

        val builder5 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
        builder5.splineTo(Vector2d(8.0, 36.0), 180.0.toRadians)
        list.add(builder5.build())

        return list
    }

    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)
        val arm = WobbleArm(hardwareMap)
        val shooter = Shooter(hardwareMap, drive)

//        val detector = UGContourRingDetector(hardwareMap, "webcam", telemetry, true)

        val zeroTrajectories = zeroTrajectories()
        val oneTrajectories = oneTrajectories()
        val fourTrajectories = fourTrajectories()

        waitForStart()

        /*val list = when (detector.height) {
            UGContourRingPipeline.Height.ZERO -> zeroTrajectories
            UGContourRingPipeline.Height.ONE -> oneTrajectories
            UGContourRingPipeline.Height.FOUR -> fourTrajectories
        }*/
        val list = zeroTrajectories

        drive.followTrajectory(list[0])
        arm.moveToDropOff()
        sleep(600)
        arm.gripOpen()
        sleep(500)
        arm.moveToCarry()

        drive.followTrajectory(list[1])
        shooter.fire()
        while (shooter.shooterState == Shooter.State.FIRING) {
            idle()
        }
        sleep(300)

        drive.followTrajectory(list[2])
        arm.gripClose()
        sleep(300)
        arm.moveToCarry()
        sleep(1000)

        drive.followTrajectory(list[3])
        arm.moveToDropOff()
        sleep(500)
        arm.gripOpen()
        sleep(500)

        drive.followTrajectory(list[4])

        val timer = ElapsedTime()
        while (!isStopRequested) {
            if (timer.seconds() > .7) {
                if (arm.armPosition == WobbleArm.ArmPosition.CARRY) {
                    arm.moveToDropOff()
                } else {
                    arm.moveToCarry()
                }
            }
        }
    }
}

val Double.toRadians get() = (Math.toRadians(this))
