package org.firstinspires.ftc.teamcode.opmodes

import com.jdroids.robotlib.command.SchedulerImpl
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Util
import org.firstinspires.ftc.teamcode.pathplanning.MotorVelocity
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive

@TeleOp(name="TeleOp")
class TeleOp : OpMode() {
    private val hangMotor1 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang1")}
    private val hangMotor2 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang2")}

    private val leftFrontMotor
            by lazy {hardwareMap.get(DcMotorEx::class.java, "lf")}
    private val leftBackMotor
            by lazy {hardwareMap.get(DcMotorEx::class.java, "lb")}

    private val rightFrontMotor
            by lazy {hardwareMap.get(DcMotorEx::class.java, "rf")}
    private val rightBackMotor
            by lazy {hardwareMap.get(DcMotorEx::class.java, "rb")}

    override fun init() {
        leftFrontMotor
        leftBackMotor

        rightFrontMotor.direction = DcMotorSimple.Direction.REVERSE
        rightBackMotor.direction = DcMotorSimple.Direction.REVERSE

        hangMotor1.direction = DcMotorSimple.Direction.REVERSE
    }

    val hangMotorPower = 0.9

    override fun loop() {
        when {
            gamepad1.left_bumper -> {
                hangMotor1.power = hangMotorPower
                hangMotor2.power = hangMotorPower
            }
            gamepad1.right_bumper -> {
                hangMotor1.power = -hangMotorPower
                hangMotor2.power = -hangMotorPower
            }
            else -> {
                hangMotor1.power = 0.0
                hangMotor2.power = 0.0
            }
        }

        curvatureDrive(
                squareWithSign(-gamepad1.left_stick_y.toDouble()),
                squareWithSign(gamepad1.right_stick_x.toDouble()),
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
            leftMotorOutput > 1.0 -> {
                rightMotorOutput -= leftMotorOutput - 1.0
                leftMotorOutput = 1.0
            }
            rightMotorOutput > 1.0 -> {
                leftMotorOutput -= rightMotorOutput - 1.0
                rightMotorOutput = 1.0
            }
            leftMotorOutput < -1.0 -> {
                rightMotorOutput -= leftMotorOutput + 1.0
                leftMotorOutput = -1.0
            }
            rightMotorOutput < -1.0 -> {
                leftMotorOutput -= rightMotorOutput + 1.0
                rightMotorOutput = -1.0
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