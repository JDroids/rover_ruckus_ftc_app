package org.firstinspires.ftc.teamcode.opmodes

import android.provider.SyncStateContract.Helpers.update
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.robotcore.internal.android.dx.dex.code.StdCatchBuilder.build
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Drive

@Disabled
@Autonomous
class SplineTestOpMode : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val dashboard = FtcDashboard.getInstance()
        val drive = Drive(this)

        val trajectory = drive.getTrajectoryBuilder()
                .splineTo(Pose2d(30.0, 30.0, 0.0))
                .waitFor(1.0)
                .reverse()
                .splineTo(Pose2d(0.0, 0.0, 0.0))
                .build()

        waitForStart()

        if (isStopRequested) return

        drive.followTrajectory(trajectory)
        while (!isStopRequested && drive.isFollowing()) {
            drive.update()
            val currentPose = drive.poseEstimate

            val packet = TelemetryPacket()
            val fieldOverlay = packet.fieldOverlay()

            packet.put("x", currentPose.x)
            packet.put("y", currentPose.y)
            packet.put("heading", currentPose.heading)

            fieldOverlay.setStrokeWidth(4)
            fieldOverlay.setStroke("green")

            fieldOverlay.setFill("blue")
            fieldOverlay.fillCircle(currentPose.x, currentPose.y, 3.0)

            dashboard.sendTelemetryPacket(packet)

            drive.update()
        }
    }
}