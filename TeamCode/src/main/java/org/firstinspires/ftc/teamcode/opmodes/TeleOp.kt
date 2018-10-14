package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp(name="TeleOp")
class TeleOp : OpMode() {
    private val leftMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "left")}
    private val rightMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "right")}

    private fun squareWithSign(value: Double) =
        if (value < 0) Math.pow(value, 2.0) else -Math.pow(value, 2.0)

    override fun init() {
        leftMotor.direction = DcMotorSimple.Direction.REVERSE
        rightMotor.direction = DcMotorSimple.Direction.FORWARD
    }

    override fun loop() {
        curvatureDrive(squareWithSign(-gamepad1.left_stick_y.toDouble()),
                squareWithSign(gamepad1.right_stick_x.toDouble()),
                gamepad1.right_stick_button)
    }

    private fun curvatureDrive(xSpeed: Double, zRotation: Double, isQuickTurn: Boolean) {
        val angularPower = if (isQuickTurn) {
            -zRotation
        }
        else {
            Math.abs(xSpeed) * -zRotation
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

        leftMotor.power = leftMotorOutput
        rightMotor.power = rightMotorOutput
        telemetry.addData("LeftMotorOutput", leftMotorOutput)
        telemetry.addData("RightMotorOutput", rightMotorOutput)
    }
}