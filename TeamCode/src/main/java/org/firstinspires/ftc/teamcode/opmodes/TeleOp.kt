package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Util

@TeleOp(name="TeleOp")
class TeleOp : OpMode() {
    private val leftMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "left")}
    private val rightMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "right")}

    private val hangMotor1 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang1")}
    private val hangMotor2 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang2")}

    private val hangServo by lazy {hardwareMap!!.get(Servo::class.java, "hangServo")}

    private fun squareWithSign(value: Double) =
        if (value < 0) Math.pow(value, 2.0) else -Math.pow(value, 2.0)

    override fun init() {
        leftMotor.direction = DcMotorSimple.Direction.REVERSE

        hangMotor2.direction = DcMotorSimple.Direction.REVERSE

        leftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
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

        when {
            gamepad1.a -> Util.setHookState(Util.HookState.LATCHED, hangServo)
            gamepad1.b -> Util.setHookState(Util.HookState.OPENED, hangServo)
        }

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