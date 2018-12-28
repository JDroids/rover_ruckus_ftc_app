package org.firstinspires.ftc.teamcode.robot.commands

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.jdroids.robotlib.command.Command
import com.jdroids.robotlib.controller.PIDControllerImpl
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.pathplanning.MotorVelocity
import org.firstinspires.ftc.teamcode.robot.Robot

class TravelFeet(private val feet: Double)
    : Command {

    @Config
    object TravelFeetCoefficients {
        @JvmField var p = 0.0
        @JvmField var i = 0.0
        @JvmField var d = 0.0
    }

    private val motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
        MotionState(0.0, 0.0),
        MotionState(feet, 0.0),
        Robot.drive.constraints.maximumVelocity,
        Robot.drive.constraints.maximumAcceleration
    )

    private val timer = ElapsedTime()

    private val pid = PIDControllerImpl(
        {getDistanceTravelled()},
        {},
        feet,
        TravelFeetCoefficients.p,
        TravelFeetCoefficients.i,
        TravelFeetCoefficients.d
    )

    override fun start() {
        Robot.drive.resetFeetChange()
        timer.reset()
    }

    override fun periodic() {
        val targetMotionState = motionProfile[timer.seconds()]

        val velocity = targetMotionState.v + pid.result()

        Robot.drive.motorVelocity = MotorVelocity(velocity, velocity)
    }

    override fun end() {
        Robot.drive.motorVelocity = MotorVelocity(0.0, 0.0)
    }

    override fun isCompleted() = Math.abs(getError()) <= 0.05

    private fun getError() = feet - getDistanceTravelled()

    private fun getDistanceTravelled() =
            (Robot.drive.leftSideFeetChange + Robot.drive.rightSideFeetChange) / 2
}