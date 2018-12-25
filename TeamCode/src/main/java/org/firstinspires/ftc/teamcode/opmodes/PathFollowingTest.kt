package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.Pose2d
import com.jdroids.robotlib.command.SchedulerImpl
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.commands.FollowTrajectory

@Autonomous(name="PathFollowingTest")
class PathFollowingTest : OpMode() {
    val INITIAL_POSITION = Pose2d()

    val goToCommand = FollowTrajectory(INITIAL_POSITION, Pose2d(0.0, 2.0))

    override fun init() {
        Robot.initHardware(this)
        Robot.drive.initialPose = INITIAL_POSITION
    }

    override fun start() {
        SchedulerImpl.run(goToCommand)
    }

    override fun loop() {
        SchedulerImpl.periodic()
    }
}