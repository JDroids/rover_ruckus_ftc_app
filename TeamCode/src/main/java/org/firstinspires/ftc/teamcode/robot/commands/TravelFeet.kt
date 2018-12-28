package org.firstinspires.ftc.teamcode.robot.commands

import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.jdroids.robotlib.command.Command
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.pathplanning.MotorVelocity
import org.firstinspires.ftc.teamcode.robot.Robot

class TravelFeet(private val feet: Double)
    : Command {

    val motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
        MotionState(0.0, 0.0),
        MotionState(feet, 0.0),
        Robot.drive.constraints.maximumVelocity,
        Robot.drive.constraints.maximumAcceleration
    )

    val timer = ElapsedTime()

    override fun start() {
        timer.reset()
    }

    override fun periodic() {
        val targetMotionState = motionProfile[timer.seconds()]

        val targetVelocity = targetMotionState.v

        Robot.drive.motorVelocity = MotorVelocity(targetVelocity, targetVelocity)
    }

    override fun end() {
        Robot.drive.motorVelocity = MotorVelocity(0.0, 0.0)
    }

    override fun isCompleted() = timer.seconds() >= motionProfile.duration()
}