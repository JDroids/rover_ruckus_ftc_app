package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.jdroids.robotlib.controller.PIDControllerImpl
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp(name="Arm Free TeleOp")
class ArmFreeTeleOp : OpMode() {
    private val hangMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang")}

    private val leftFrontMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "lf")}
    private val leftBackMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "lb")}

    private val rightFrontMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "rf")}

    private val rightBackMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "rb")}

    override fun init() {
        rightFrontMotor.direction = DcMotorSimple.Direction.REVERSE
        rightBackMotor.direction = DcMotorSimple.Direction.REVERSE
    }

    private val hangMotorPower = 0.9

    override fun loop() {
        // Deal with hang mechanism (left bumper to retract, right bumper to extend)
        when {
            gamepad1.left_bumper -> {
                hangMotor.power = -hangMotorPower
            }
            gamepad1.right_bumper -> {
                hangMotor.power = hangMotorPower
            }
            else -> {
                hangMotor.power = 0.0
            }
        }
        // Deal with drivetrain
        curvatureDrive(
                squareWithSign(gamepad1.left_stick_y.toDouble()),
                squareWithSign(- gamepad1.right_stick_x.toDouble()),
                gamepad1.right_stick_button
        )
    }

    private fun curvatureDrive(xSpeed: Double, zRotation: Double, isQuickTurn: Boolean) {
        val angularPower = if (isQuickTurn) {
            zRotation
        }
        else {
            Math.abs(xSpeed) * zRotation
        }

        var leftMotorOutput = xSpeed + angularPower
        var rightMotorOutput = xSpeed - angularPower

        //If rotation is overpowered, reduce both outputs to allowable range

        when {
            leftMotorOutput > 0.85 -> {
                rightMotorOutput -= leftMotorOutput - 0.85
                leftMotorOutput = 0.85
            }
            rightMotorOutput > 0.85 -> {
                leftMotorOutput -= rightMotorOutput - 0.85
                rightMotorOutput = 0.85
            }
            leftMotorOutput < -0.85 -> {
                rightMotorOutput -= leftMotorOutput + 0.85
                leftMotorOutput = -0.85
            }
            rightMotorOutput < -0.85 -> {
                leftMotorOutput -= rightMotorOutput + 0.85
                rightMotorOutput = -0.85
            }
        }

        leftFrontMotor.power = leftMotorOutput
        leftBackMotor.power = leftMotorOutput

        rightFrontMotor.power = rightMotorOutput
        rightBackMotor.power = rightMotorOutput
    }

    private fun squareWithSign(value: Double) =
            if (value < 0) Math.pow(value, 2.0) else -Math.pow(value, 2.0)

}