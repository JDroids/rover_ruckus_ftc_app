package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.profile.MotionProfileBuilder
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Drive
import org.firstinspires.ftc.teamcode.DriveConstants
import org.firstinspires.ftc.teamcode.Util

@Disabled
@TeleOp(name="Path Follower Test")
class PathFollowerTest : LinearOpMode() {
    private val leftFrontMotor
            by lazy { hardwareMap.get(DcMotorEx::class.java, "lf") }
    private val leftBackMotor
            by lazy { hardwareMap.get(DcMotorEx::class.java, "lb") }

    private val rightFrontMotor
            by lazy { hardwareMap.get(DcMotorEx::class.java, "rf") }
    private val rightBackMotor
            by lazy { hardwareMap.get(DcMotorEx::class.java, "rb") }

    val profile = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(0.0, 0.0, 0.0, 0.0),
            MotionState(60.0, 0.0, 0.0, 0.0),
            DriveConstants.BASE_CONSTRAINTS.maximumVelocity,
            DriveConstants.BASE_CONSTRAINTS.maximumAcceleration
    )

    override fun runOpMode() {
        rightFrontMotor.direction = DcMotorSimple.Direction.REVERSE
        rightBackMotor.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        Util.followProfile(profile, this, leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor)
    }
}