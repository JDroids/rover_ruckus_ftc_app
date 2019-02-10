package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.pathplanning.LinearPathFollower
import org.firstinspires.ftc.teamcode.pathplanning.MotionProfilingConstraints

@TeleOp(name="Linear Path Follower Test")
class LinearPathFollowerTest : LinearOpMode() {
    override fun runOpMode() {
        val motor = hardwareMap.get(DcMotorEx::class.java, "motor")

        waitForStart()

        val follower = LinearPathFollower(2.0, MotionProfilingConstraints(1.5, 0.5))

        val timer = ElapsedTime()

        while (opModeIsActive()) {
            val target = follower.generate(timer.seconds()).state.linearVelocity

            motor.setVelocity(target * Math.PI * 2, AngleUnit.RADIANS)
        }

        motor.power = 0.0
    }
}