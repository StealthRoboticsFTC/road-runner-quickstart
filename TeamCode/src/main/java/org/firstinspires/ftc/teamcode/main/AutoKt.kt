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
import java.util.*

@Autonomous
class AutoKt: LinearOpMode() {
    enum class AutoType {
        ZERO,
        ONE,
        FOUR
    }

    @Config
    companion object {
        @JvmStatic
        var autoType: AutoType = AutoType.ZERO
    }

    private val slowConstraints = DriveConstraints(5.0, BASE_CONSTRAINTS.maxAccel, BASE_CONSTRAINTS.maxJerk, BASE_CONSTRAINTS.maxAngVel, BASE_CONSTRAINTS.maxAngAccel, BASE_CONSTRAINTS.maxAngJerk)
    private val slowCombinedConstraints = MecanumConstraints(slowConstraints, TRACK_WIDTH)

    private val startPose = Pose2d(-60.0, 19.0, 0.0)
    
    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)
        val arm = WobbleArm(hardwareMap)
        val shooter = Shooter(hardwareMap, drive)

        val list = ArrayList<Trajectory>()

        val builder3 = drive.trajectoryBuilder(list[list.size - 1].end(), 90.0.toRadians)
        builder3
                .splineToSplineHeading(Pose2d(-28.0, 57.0, 180.0.toRadians), 180.0.toRadians)
                .addTemporalMarker(0.0, -1.5) { arm.moveToPickup() }
                .splineTo(Vector2d(-36.0, 57.0), 180.0.toRadians, slowCombinedConstraints)

        when (autoType) {
            AutoType.ZERO -> {
                val builder1 = drive.trajectoryBuilder(startPose, startPose.heading)
                builder1.splineTo(Vector2d(22.0, 40.0), 90.0.toRadians)
                list.add(builder1.build())

                val builder2 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
                builder2.splineTo(Vector2d(-20.0, 36.0), 180.0.toRadians)
                list.add(builder2.build())

                list.add(builder3.build())

                val builder4 = drive.trajectoryBuilder(list[list.size - 1].end(), 0.0.toRadians)
                builder4.splineToSplineHeading(Pose2d(-20.0, 57.0, 0.0.toRadians), 0.0.toRadians)
                builder4.splineTo(Vector2d(-6.0, 57.0), 0.0.toRadians)
                list.add(builder4.build())

                val builder5 = drive.trajectoryBuilder(list[list.size - 1].end(), 270.0.toRadians)
                builder5.splineToConstantHeading(Vector2d(8.0, 36.0), 0.0.toRadians)
                list.add(builder5.build())
            }

            AutoType.ONE -> {
                val builder1 = drive.trajectoryBuilder(startPose, startPose.heading)
                builder1.splineTo(Vector2d(26.0, 26.0), 0.0.toRadians)
                list.add(builder1.build())

                val builder2 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
                builder2.splineTo(Vector2d(-20.0, 36.0), 180.0.toRadians)
                list.add(builder2.build())

                list.add(builder3.build())

                val builder4 = drive.trajectoryBuilder(list[list.size - 1].end(), 0.0.toRadians)
                builder4.splineToSplineHeading(Pose2d(-26.0, 57.0, 0.0.toRadians), 0.0.toRadians)
                builder4.splineTo(Vector2d(19.0, 26.0), 0.0.toRadians)
                list.add(builder4.build())

                val builder5 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
                builder5.back(10.0)
                list.add(builder5.build())
            }

            AutoType.FOUR -> {
                val builder1 = drive.trajectoryBuilder(startPose, startPose.heading)
                builder1.splineTo(Vector2d(56.0, 40.0), 60.0.toRadians)
                list.add(builder1.build())

                val builder2 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
                builder2.splineTo(Vector2d(-20.0, 36.0), 180.0.toRadians)
                list.add(builder2.build())

                list.add(builder3.build())

                val builder4 = drive.trajectoryBuilder(list[list.size - 1].end(), 0.0.toRadians)
                builder4.splineToSplineHeading(Pose2d(-20.0, 57.0, 0.0.toRadians), 0.0.toRadians)
                builder4.splineTo(Vector2d(46.0, 57.0), 0.0.toRadians)
                list.add(builder4.build())

                val builder5 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
                builder5.splineTo(Vector2d(8.0, 36.0), 180.0.toRadians)
                list.add(builder5.build())
            }
        }

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
