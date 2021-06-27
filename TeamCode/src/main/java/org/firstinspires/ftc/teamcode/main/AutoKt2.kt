package org.firstinspires.ftc.teamcode.main

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.EPSILON
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL
import org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystem.AutonomousAiming
import org.firstinspires.ftc.teamcode.subsystem.Intake
import org.firstinspires.ftc.teamcode.subsystem.Shooter
import org.firstinspires.ftc.teamcode.subsystem.WobbleArm
import org.firstinspires.ftc.teamcode.vision.UGContourRingDetector
import org.firstinspires.ftc.teamcode.vision.UGContourRingPipeline

@Autonomous
class AutoKt2: LinearOpMode() {
    private val accelConstraint = ProfileAccelerationConstraint(MAX_ACCEL)

    private val intakeVelConstraint = MecanumVelocityConstraint(8.0, TRACK_WIDTH)
    private val slowVelConstraint = MecanumVelocityConstraint(15.0, TRACK_WIDTH)

    private val startPose = Pose2d(-63.0, 19.0, 0.0.toRadians)
    
    private lateinit var drive: SampleMecanumDrive
    private lateinit var arm: WobbleArm
    private lateinit var shooter: Shooter
    private lateinit var intake: Intake
//    private lateinit var aiming: AutonomousAiming

    private fun update() {
        drive.update()
        shooter.update()
//        aiming.update()
    }

    private fun sleepUpdate(ms: Long) {
        val elapsedTime = ElapsedTime()
        while (elapsedTime.milliseconds() < ms && !isStopRequested) {
            update()
        }
    }
    
    private fun followTrajectory(trajectory: Trajectory) {
        drive.followTrajectoryAsync(trajectory)
        
        while (drive.isBusy && !isStopRequested) {
            update()
        }
    }

    private fun turn(angle: Double) {
        drive.turnAsync(angle)

        while (drive.isBusy && !isStopRequested) {
            update()
        }
    }

    private fun getTrajsFour(): List<Trajectory> {
        val list = mutableListOf<Trajectory>()

        val builder1 = drive.trajectoryBuilder(startPose)
                .splineTo(Vector2d(-5.0, 12.0), 356.0.toRadians)
                .addTemporalMarker(1.0, -1.0, shooter::startPowershotRampUp)
        list.add(builder1.build())

        val builder2 = drive.trajectoryBuilder(list[list.size - 1].end() + Pose2d(0.0, 0.0, 12.0.toRadians))
                .addTemporalMarker(0.2, shooter::stop)
                .splineTo(Vector2d(56.0, 48.0), 45.0.toRadians)
                .addTemporalMarker(1.0, -2.0, arm::moveToDropOff)
                .addDisplacementMarker(arm::gripOpen)
        list.add(builder2.build())

        val builder3 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
                .addTemporalMarker(1.0, arm::moveToCarry)
                .splineTo(Vector2d(6.0, 36.0), 180.0.toRadians)
                .addDisplacementMarker(20.0, intake::moveStackArmOut)
                .splineTo(Vector2d(-10.0, 36.0), 180.0.toRadians)
                .addDisplacementMarker(intake::startIn)
//                .splineTo(Vector2d(-26.0, 36.0), 180.0.toRadians)
                .splineTo(Vector2d(-17.0, 36.0), 180.0.toRadians, intakeVelConstraint, accelConstraint)
        list.add(builder3.build())

//        val builder4 = drive.trajectoryBuilder(list[list.size - 1].end())
//                .splineTo(Vector2d(-22.0, 36.0), 0.0.toRadians)
//        list.add(builder4.build())
        //REMOVE IF RE-ADD OTHER TRAJ

//        val builder5 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
//                .splineTo(Vector2d(-24.0, 36.0), 180.0.toRadians)
//                .addDisplacementMarker(intake::startIn)
//                .splineTo(Vector2d(-30.0, 36.0), 180.0.toRadians, intakeVelConstraint, accelConstraint)
//        list.add(builder5.build())
        list.add(builder3.build())
        list.add(builder3.build())

        val builder6 = drive.trajectoryBuilder(list[list.size - 1].end())
                .addTemporalMarker(1.0, intake::stop)
                .splineTo(Vector2d(-3.0, 36.0), 0.0.toRadians)
                .addTemporalMarker(1.0, -1.5, shooter::startRampUp)
        list.add(builder6.build())

        val builder7 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
                .addTemporalMarker(0.2, shooter::stop)
                .splineTo(Vector2d(-16.0, 36.0), 180.0.toRadians)
                .addDisplacementMarker(intake::startIn)
                .splineTo(Vector2d(-34.0, 36.0), 180.0.toRadians, intakeVelConstraint, accelConstraint)
        list.add(builder7.build())

        val builder8 = drive.trajectoryBuilder(list[list.size - 1].end())
                .splineTo(Vector2d(-3.0, 36.0), 0.0.toRadians)
                .addTemporalMarker(1.0, -1.5, shooter::startRampUp)
                .addTemporalMarker(1.0, -1.0, intake::stop)
        list.add(builder8.build())

        val builder9 = drive.trajectoryBuilder(list[list.size - 1].end(), 110.0.toRadians)
                .addTemporalMarker(0.2, shooter::stop)
                .addTemporalMarker(1.0, arm::moveToPickup)
                .splineToSplineHeading(Pose2d(-28.0, 53.5, 180.0.toRadians - EPSILON), 180.0.toRadians)
                .splineTo(Vector2d(-38.0, 53.5), 180.0.toRadians, slowVelConstraint, accelConstraint)
                .addTemporalMarker(1.0, -0.3, arm::gripClose)
        list.add(builder9.build())

        val builder10 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
                .splineToSplineHeading(Pose2d(-22.0, 53.5, 0.0.toRadians - 2.0 * EPSILON), 0.0.toRadians)
                .splineTo(Vector2d(39.0, 56.0), 5.0.toRadians)
                .addTemporalMarker(1.0, -1.5, arm::moveToDropOff)
                .addDisplacementMarker(arm::gripOpen)
        list.add(builder10.build())

        val builder11 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
                .addTemporalMarker(1.0, arm::moveToInitial)
                .splineTo(Vector2d(12.0, 53.0), 180.0.toRadians)
        list.add(builder11.build())

        return list
    }

    private fun runFour(list: List<Trajectory>) {
        followTrajectory(list[0])

        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        turn(Angle.normDelta(356.0.toRadians - drive.poseEstimate.heading))
        shooter.fire(1)
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        sleepUpdate(600)
        turn(Angle.normDelta(2.0.toRadians - drive.poseEstimate.heading))
        shooter.fire(1)
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        sleepUpdate(600)
        turn(Angle.normDelta(8.0.toRadians - drive.poseEstimate.heading))
        shooter.fire(1)
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        sleepUpdate(600)

        followTrajectory(list[1])

        followTrajectory(list[2])
//        followTrajectory(list[3])
//
//        followTrajectory(list[4])

        followTrajectory(list[5])
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        shooter.fire(1)
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        sleepUpdate(600)

        followTrajectory(list[6])

        followTrajectory(list[7])
        intake.moveStackArmIn()
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        shooter.fire(3)
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        sleepUpdate(600)

        followTrajectory(list[8])
//        sleepUpdate(100)
        arm.moveToCarry()

        followTrajectory(list[9])

        intake.moveStackArmOut()

        followTrajectory(list[10])
    }

    private fun getTrajsOne(): List<Trajectory> {
        val list = mutableListOf<Trajectory>()

        val builder1 = drive.trajectoryBuilder(startPose)
                .splineTo(Vector2d(-5.0, 12.0), 356.0.toRadians)
                .addTemporalMarker(1.0, -1.0, shooter::startPowershotRampUp)
        list.add(builder1.build())

        val builder2 = drive.trajectoryBuilder(list[list.size - 1].end() + Pose2d(0.0, 0.0, 12.0.toRadians))
                .addTemporalMarker(0.2, shooter::stop)
                .splineTo(Vector2d(25.0, 34.0), 20.0.toRadians)
                .addTemporalMarker(1.0, -2.0, arm::moveToDropOff)
                .addDisplacementMarker(arm::gripOpen)
        list.add(builder2.build())

        val builder3 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
                .addTemporalMarker(1.0, intake::startIn)
                .splineTo(Vector2d(-22.0, 36.0), 180.0.toRadians)
        list.add(builder3.build())

        val builder4 = drive.trajectoryBuilder(list[list.size - 1].end())
                .addTemporalMarker(1.0, intake::stop)
                .splineTo(Vector2d(-3.0, 36.0), 0.0.toRadians)
                .addTemporalMarker(1.0, -1.5, shooter::startRampUp)
        list.add(builder4.build())

        val builder5 = drive.trajectoryBuilder(list[list.size - 1].end(), 120.0.toRadians)
                .addTemporalMarker(0.2, shooter::stop)
                .addTemporalMarker(1.0, arm::moveToPickup)
                .splineToSplineHeading(Pose2d(-28.0, 53.5, 180.0.toRadians - EPSILON), 180.0.toRadians)
                .splineTo(Vector2d(-36.0, 53.5), 180.0.toRadians, slowVelConstraint, accelConstraint)
                .addTemporalMarker(1.0, -0.3, arm::gripClose)
        list.add(builder5.build())

        val builder6 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
                .splineToSplineHeading(Pose2d(-22.0, 53.5, 0.0.toRadians - 2.0 * EPSILON), 0.0.toRadians)
                .splineTo(Vector2d(14.0, 39.0), 0.0.toRadians)
                .addTemporalMarker(1.0, -1.5, arm::moveToDropOff)
                .addDisplacementMarker(arm::gripOpen)
        list.add(builder6.build())

        val builder7 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
                .addTemporalMarker(1.0, arm::moveToInitial)
                .splineTo(Vector2d(10.0, 39.0), 180.0.toRadians)
        list.add(builder7.build())

        return list
    }

    private fun runOne(list: List<Trajectory>) {
        followTrajectory(list[0])

        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        turn(Angle.normDelta(356.0.toRadians - drive.poseEstimate.heading))
        shooter.fire(1)
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        sleepUpdate(600)
        turn(Angle.normDelta(2.0.toRadians - drive.poseEstimate.heading))
        shooter.fire(1)
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        sleepUpdate(600)
        turn(Angle.normDelta(8.0.toRadians - drive.poseEstimate.heading))
        shooter.fire(1)
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        sleepUpdate(600)

        followTrajectory(list[1])

        followTrajectory(list[2])

        followTrajectory(list[3])
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        shooter.fire(1)
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        sleepUpdate(600)

        followTrajectory(list[4])
//        sleepUpdate(100)
        arm.moveToCarry()

        followTrajectory(list[5])

        followTrajectory(list[6])
    }

    private fun getTrajsZero(): List<Trajectory> {
        val list = mutableListOf<Trajectory>()

        val builder1 = drive.trajectoryBuilder(startPose)
                .splineTo(Vector2d(-5.0, 12.0), 356.0.toRadians)
                .addTemporalMarker(1.0, -1.0, shooter::startPowershotRampUp)
        list.add(builder1.build())

        val builder2 = drive.trajectoryBuilder(list[list.size - 1].end() + Pose2d(0.0, 0.0, 12.0.toRadians))
                .addTemporalMarker(0.2, shooter::stop)
                .splineTo(Vector2d(17.0, 50.0), 90.0.toRadians)
                .addTemporalMarker(1.0, -2.0, arm::moveToDropOff)
                .addDisplacementMarker(arm::gripOpen)
        list.add(builder2.build())

        val builder3 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
                .addTemporalMarker(0.2, shooter::stop)
                .addTemporalMarker(1.0, arm::moveToPickup)
                .splineToSplineHeading(Pose2d(-28.0, 54.5, 180.0.toRadians - EPSILON), 180.0.toRadians)
                .splineTo(Vector2d(-36.0, 54.5), 180.0.toRadians, slowVelConstraint, accelConstraint)
                .addTemporalMarker(1.0, -0.3, arm::gripClose)
        list.add(builder3.build())

        val builder4 = drive.trajectoryBuilder(list[list.size - 1].end(), true)
                .splineToSplineHeading(Pose2d(-10.0, 63.0, 0.0.toRadians - 2.0 * EPSILON), 0.0.toRadians)
                .addTemporalMarker(1.0, -1.5, arm::moveToDropOff)
                .addDisplacementMarker(arm::gripOpen)
        list.add(builder4.build())

        val builder5 = drive.trajectoryBuilder(list[list.size - 1].end(), 240.0.toRadians)
                .addTemporalMarker(1.0, arm::moveToInitial)
                .splineToConstantHeading(Vector2d(10.0, 36.0), 0.0.toRadians)
        list.add(builder5.build())

        return list
    }

    private fun runZero(list: List<Trajectory>) {
        followTrajectory(list[0])

        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        turn(Angle.normDelta(356.0.toRadians - drive.poseEstimate.heading))
        shooter.fire(1)
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        sleepUpdate(600)
        turn(Angle.normDelta(2.0.toRadians - drive.poseEstimate.heading))
        shooter.fire(1)
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        sleepUpdate(600)
        turn(Angle.normDelta(8.0.toRadians - drive.poseEstimate.heading))
        shooter.fire(1)
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        sleepUpdate(600)

        followTrajectory(list[1])

        followTrajectory(list[2])
        arm.moveToCarry()

        followTrajectory(list[3])

        followTrajectory(list[4])
    }

    override fun runOpMode() {
        drive = SampleMecanumDrive(hardwareMap)
        arm = WobbleArm(hardwareMap)
        intake = Intake(hardwareMap)
        shooter = Shooter(hardwareMap, intake)
//        aiming = AutonomousAiming(hardwareMap, drive, shooter)

        arm.gripClose()

        val detector = UGContourRingDetector(hardwareMap, "webcam", telemetry, true)

        detector.init()

        val trajsFour = getTrajsFour()
        val trajsOne = getTrajsOne()
        val trajsZero = getTrajsZero()

        waitForStart()
        drive.poseEstimate = startPose

        val height = detector.height

        detector.camera.closeCameraDeviceAsync {  }

        when (height) {
            UGContourRingPipeline.Height.ZERO -> runZero(trajsZero)
            UGContourRingPipeline.Height.ONE -> runOne(trajsOne)
            UGContourRingPipeline.Height.FOUR -> runFour(trajsFour)
        }

        if (!isStopRequested) {
            intake.moveStackArmIn()
        }

        PoseStorage.pose = drive.poseEstimate

        sleepUpdate(1000)
    }
}