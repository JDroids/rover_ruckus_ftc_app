package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.jdroids.robotlib.controller.PIDControllerImpl
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDCoefficients
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp(name="TeleOp")
class TeleOp : OpMode() {
    private val hangMotor1 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang1")}
    private val hangMotor2 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang2")}

    private val leftFrontMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "lf")}
    private val leftBackMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "lb")}

    private val rightFrontMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "rf")}

    private val rightBackMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "rb")}

    private val leftArmMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "leftArmMotor")}
    private val rightArmMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "rightArmMotor")}

    private val armTOFSensor by lazy {hardwareMap.get(Rev2mDistanceSensor::class.java, "tofSensor")}

    @Config
    object ArmPIDCoefficients {
        @JvmField var ARM_PID = PIDCoefficients(0.0, 0.0, 0.0)
    }

    private val armPID = PIDControllerImpl(
        {armTOFSensor.getDistance(DistanceUnit.INCH)},
        {},
        2.5,
        ArmPIDCoefficients.ARM_PID.p, ArmPIDCoefficients.ARM_PID.i, ArmPIDCoefficients.ARM_PID.d
    )

    override fun init() {
        leftFrontMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        leftBackMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        rightFrontMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        rightBackMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        rightFrontMotor.direction = DcMotorSimple.Direction.REVERSE
        rightBackMotor.direction = DcMotorSimple.Direction.REVERSE

        hangMotor1.direction = DcMotorSimple.Direction.REVERSE
    }

    private val hangMotorPower = 0.9
    private val armMotorPower = 0.2

    override fun loop() {
        val targetArmPower = armPID.result()

        // Deal with hang mechanism (left bumper to retract, right bumper to extend)
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

        // Deal with arm
        if (gamepad2.a) {
            leftArmMotor.power = -targetArmPower
            rightArmMotor.power = targetArmPower
        }
        else {
            when {
                gamepad2.left_bumper -> {
                    if (armTOFSensor.getDistance(DistanceUnit.INCH) > 2.5) {
                        leftArmMotor.power = 0.2
                        rightArmMotor.power = -0.2
                    }
                }
                gamepad2.right_bumper -> {
                    leftArmMotor.power = -0.8
                    rightArmMotor.power = 0.8
                }
                else -> {
                    leftArmMotor.power = 0.0
                    rightArmMotor.power = 0.0
                }
            }
        }

        FtcDashboard.getInstance().telemetry.addData("target power", targetArmPower)
        FtcDashboard.getInstance().telemetry.addData("sensor output", armTOFSensor.getDistance(DistanceUnit.INCH))

        // Deal with drivetrain
        curvatureDrive(
                squareWithSign(-gamepad1.left_stick_y.toDouble()),
                squareWithSign(gamepad1.right_stick_x.toDouble()),
                gamepad1.right_stick_button
        )

        FtcDashboard.getInstance().telemetry.update()
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