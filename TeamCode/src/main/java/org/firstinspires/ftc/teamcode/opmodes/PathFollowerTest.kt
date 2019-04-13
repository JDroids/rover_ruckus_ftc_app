package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Drive

@TeleOp(name="Path Follower Test")
class PathFollowerTest : LinearOpMode() {
    override fun runOpMode() {
        val drive = Drive(this)
        val trajectory = drive.getTrajectoryBuilder()
                    .lineTo(Vector2d(10.0, 10.0))
                    .turn(Math.PI/2)
                    .build()

        waitForStart()

        drive.followTrajectory(trajectory)

        while (drive.isFollowing() && opModeIsActive()) {
            drive.update()
        }
    }
}