package org.firstinspires.ftc.teamcode.robot.commands

import com.jdroids.robotlib.command.Command
import com.jdroids.robotlib.command.SchedulerImpl
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.pathplanning.*
import org.firstinspires.ftc.teamcode.robot.Robot

class FollowConstantCurvaturePath(path: ConstantCurvaturePath) : Command {
    override fun isCompleted() = elapsedTime.seconds() > constantCurvaturePathFollower.timeToFollow


    private val constantCurvaturePathFollower =
            ConstantCurvaturePathFollower(path, Robot.drive.constraints, Robot.drive.statistics)

    private val elapsedTime = ElapsedTime()

    override fun start() {
        elapsedTime.reset()
    }

    override fun periodic() {
        val target = constantCurvaturePathFollower.generate(elapsedTime.seconds())

        Robot.drive.motorVelocity = target.state.toMotorVelocity(Robot.drive.statistics)
    }

    override fun end() {
        Robot.drive.motorVelocity = MotorVelocity(0.0, 0.0)
    }
}