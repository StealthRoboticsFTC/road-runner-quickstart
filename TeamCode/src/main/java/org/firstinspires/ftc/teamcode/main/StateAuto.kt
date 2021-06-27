package org.firstinspires.ftc.teamcode.main

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.EPSILON
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystem.Intake
import org.firstinspires.ftc.teamcode.subsystem.Shooter
import org.firstinspires.ftc.teamcode.subsystem.WobbleArm
import org.firstinspires.ftc.teamcode.vision.UGContourRingDetector
import org.firstinspires.ftc.teamcode.vision.UGContourRingPipeline

@Autonomous
class StateAuto: LinearOpMode() {
    private val accelConstraint = ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)

    private val intakeVelConstraint = MecanumVelocityConstraint(8.0, DriveConstants.TRACK_WIDTH)
    private val slowVelConstraint = MecanumVelocityConstraint(15.0, DriveConstants.TRACK_WIDTH)

    private val startPose = Pose2d(-63.0, 32.0, 0.0.toRadians)

    private lateinit var drive: SampleMecanumDrive
    private lateinit var arm: WobbleArm
    private lateinit var shooter: Shooter
    private lateinit var intake: Intake
    private lateinit var detector: UGContourRingDetector
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

    private fun runPS() {
        shooter.setFlapPosition(0.64)
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        turn(Angle.normDelta(349.5.toRadians - drive.poseEstimate.heading))
        shooter.fire(1)
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        sleepUpdate(300)
        turn(Angle.normDelta(345.0.toRadians - drive.poseEstimate.heading))
        shooter.fire(1)
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        sleepUpdate(300)
        shooter.setFlapPosition(0.65)
        turn(Angle.normDelta(339.0.toRadians - drive.poseEstimate.heading))
        shooter.fire(1)
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        sleepUpdate(300)
    }

    private fun getTrajsZero(): List<Trajectory> {
        val list = mutableListOf<Trajectory>()

        val builder1 = drive.trajectoryBuilder(startPose, startPose.heading)
                .splineTo(Vector2d(-4.0, 35.0), 352.0.toRadians)
        list.add(builder1.build())

        val builder2 = drive.trajectoryBuilder(list.get(list.size - 1).end() + Pose2d(0.0, 0.0, (-11.0).toRadians))
                .splineTo(Vector2d(14.0, 48.0), 75.0.toRadians)
        list.add(builder2.build())

        val builder3 = drive.trajectoryBuilder(list.get(list.size - 1).end(), 180.0.toRadians)
                .splineToSplineHeading(Pose2d(-38.0, 58.0, 180.0.toRadians), (180.0).toRadians)
                .addTemporalMarker(1.0, -1.0, arm::moveToPickup)
        list.add(builder3.build())

        val builder4 = drive.trajectoryBuilder(list.get(list.size - 1).end(), true)
                .splineToSplineHeading(Pose2d(-10.0, 58.0, 0.0.toRadians - EPSILON), 0.0.toRadians)
                .addTemporalMarker(1.0, -0.6, arm::moveToDropOff)
        list.add(builder4.build())

        val builder5 = drive.trajectoryBuilder(list.get(list.size - 1).end(), 270.0.toRadians)
                .splineToConstantHeading(Vector2d(10.0, 8.0), 325.0.toRadians)
        list.add(builder5.build())

        return list
    }

    private fun runZero(list: List<Trajectory>) {
        shooter.startPowershotRampUp()

        followTrajectory(list[0])

        runPS()
        shooter.stop()
        arm.moveToDropOff()

        followTrajectory(list[1])

        arm.gripOpen()

        followTrajectory(list[2])

        arm.gripClose()
        sleepUpdate(400)
        arm.moveToCarry()

        followTrajectory(list[3])

        arm.gripOpen()
        sleepUpdate(400)

        followTrajectory(list[4])

        arm.moveToInitial()
        sleepUpdate(2000)
    }

    private fun getTrajsOne(): List<Trajectory> {
        val list = mutableListOf<Trajectory>()

        val builder1 = drive.trajectoryBuilder(startPose)
                .splineTo(Vector2d(-36.0, 36.0), 0.0.toRadians)
        list.add(builder1.build())

        val builder2 = drive.trajectoryBuilder(list.get(list.size - 1).end())
                .splineTo(Vector2d(-4.0, 35.0), 352.0.toRadians)
                .addDisplacementMarker(1.0, -2.0, intake::stop)
        list.add(builder2.build())

        val builder3 = drive.trajectoryBuilder(list.get(list.size - 1).end() + Pose2d(0.0, 0.0, (-11.0).toRadians))
                .splineTo(Vector2d(23.0, 32.0), 0.0.toRadians)
        list.add(builder3.build())

        val builder4 = drive.trajectoryBuilder(list.get(list.size - 1).end(), 160.0.toRadians)
                .splineToSplineHeading(Pose2d(-38.0, 58.0, 180.0.toRadians + EPSILON), 180.0.toRadians)
                .addTemporalMarker(1.0, -1.0, arm::moveToPickup)
        list.add(builder4.build())

        val builder5 = drive.trajectoryBuilder(list.get(list.size - 1).end(), 340.0.toRadians)
                .splineToSplineHeading(Pose2d(11.0, 30.0, 0.0.toRadians + EPSILON), 340.0.toRadians)
                .addTemporalMarker(1.0, -0.6, arm::moveToDropOff)
        list.add(builder5.build())

        val builder6 = drive.trajectoryBuilder(list.get(list.size - 1).end(), 270.0.toRadians)
                .splineToConstantHeading(Vector2d(6.0, 8.0), 240.0.toRadians)
        list.add(builder6.build())

        return list
    }
    
    private fun runOne(trajs: List<Trajectory>) {
        shooter.startRampUp()
        shooter.setFlapPosition(0.57)

        followTrajectory(trajs[0])

        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }

        turn(Angle.normDelta(1.0.toRadians - drive.poseEstimate.heading))
        shooter.fire(1)

        sleepUpdate(300)

        shooter.startPowershotRampUp()
        intake.startIn()

        followTrajectory(trajs[1])

        runPS()
        shooter.stop()
        arm.moveToDropOff()

        followTrajectory(trajs[2])

        arm.gripOpen()

        followTrajectory(trajs[3])

        arm.gripClose()
        sleepUpdate(400)
        arm.moveToCarry()

        followTrajectory(trajs[4])

        arm.gripOpen()
        sleepUpdate(400)

        followTrajectory(trajs[5])

        arm.moveToInitial()
        arm.gripClose()
        sleepUpdate(2000)
    }

    private fun getTrajsFour(): List<Trajectory> {
        val list = mutableListOf<Trajectory>()

        val builder1 = drive.trajectoryBuilder(startPose)
                .splineTo(Vector2d(-40.0, 36.0), 0.0.toRadians)
        list.add(builder1.build())

        val builder2 = drive.trajectoryBuilder(list.get(list.size - 1).end())
                .splineTo(Vector2d(-22.0, 36.0), 0.0.toRadians)
        list.add(builder2.build())

        val builder3 = drive.trajectoryBuilder(list.get(list.size - 1).end())
                .splineTo(Vector2d(-4.0, 36.0), 352.0.toRadians)
        list.add(builder3.build())

        val builder4 = drive.trajectoryBuilder(list.get(list.size - 1).end() + Pose2d(0.0, 0.0, (-11.0).toRadians))
                .splineTo(Vector2d(52.0, 48.0), 30.0.toRadians)
                .addTemporalMarker(1.0, -0.5, arm::gripOpen)
        list.add(builder4.build())

        val builder5 = drive.trajectoryBuilder(list.get(list.size - 1).end(), 180.0.toRadians)
                .splineToSplineHeading(Pose2d(-36.0, 59.0, 180.0.toRadians), 180.0.toRadians)
                .addTemporalMarker(1.0, -1.2, arm::moveToPickup)
        list.add(builder5.build())

        val builder6 = drive.trajectoryBuilder(list.get(list.size - 1).end(), true)
                .splineToSplineHeading(Pose2d(38.0, 58.0, 0.0.toRadians - EPSILON), 0.0.toRadians)
                .addTemporalMarker(1.0, -1.0, arm::moveToDropOff)
                .addTemporalMarker(1.0, -0.3, arm::gripOpen)
        list.add(builder6.build())

        val builder7 = drive.trajectoryBuilder(list.get(list.size - 1).end(), 270.0.toRadians)
                .splineToConstantHeading(Vector2d(4.0, 12.0), 200.0.toRadians)
        list.add(builder7.build())

        return list
    }

    private fun runFour(list: List<Trajectory>) {
        shooter.startRampUp()
        shooter.setFlapPosition(0.57)

        intake.moveStackArmOut()

        followTrajectory(list[0])

        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }

        turn(Angle.normDelta(2.0.toRadians - drive.poseEstimate.heading))
        shooter.fire(3)
        while (shooter.state != Shooter.State.RUNNING && !isStopRequested) {
            update()
        }
        sleepUpdate(300)
        intake.startIn()

        shooter.setFlapPosition(0.57)

        followTrajectory(list[1])

        turn(Angle.normDelta(2.0.toRadians - drive.poseEstimate.heading))
        shooter.fire(1)
        sleepUpdate(300)
        shooter.startPowershotRampUp()

        followTrajectory(list[2])

        intake.stop()
        intake.moveStackArmIn()
        runPS()
        shooter.stop()
        arm.moveToDropOff()

        followTrajectory(list[3])

        arm.moveToCarry()

        followTrajectory(list[4])

        arm.gripClose()
        sleepUpdate(500)
        arm.moveToCarry()

        followTrajectory(list[5])

        arm.moveToInitial()

        followTrajectory(list[6])

        arm.gripClose()
    }

    override fun runOpMode() {
        drive = SampleMecanumDrive(hardwareMap)
        intake = Intake(hardwareMap)
        shooter = Shooter(hardwareMap, intake)
        arm = WobbleArm(hardwareMap)
        detector = UGContourRingDetector(hardwareMap, "Webcam 2", telemetry, true)

        detector.init()

        val trajsZero = getTrajsZero()
        val trajsOne = getTrajsOne()
        val trajsFour = getTrajsFour()

        waitForStart()
        drive.poseEstimate = startPose

        val height = detector.height

        detector.camera.closeCameraDeviceAsync {  }

        when (height) {
            UGContourRingPipeline.Height.ZERO -> runZero(trajsZero)
            UGContourRingPipeline.Height.ONE -> runOne(trajsOne)
            UGContourRingPipeline.Height.FOUR -> runFour(trajsFour)
        }

        PoseStorage.pose = drive.poseEstimate

        sleepUpdate(1000)
    }
}