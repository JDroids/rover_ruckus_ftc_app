package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.jdroids.robotlib.controller.PIDControllerImpl
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
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

    private val armPotent by lazy {hardwareMap.get(AnalogInput::class.java, "potent")}
    private val spinner by lazy {hardwareMap.get(CRServo::class.java, "intakeServo")}
    private val gate by lazy {hardwareMap.get(Servo::class.java, "gate")}
    private val elbow by lazy {hardwareMap.get(Servo::class.java, "elbow")}
    private val wrist by lazy {hardwareMap.get(Servo::class.java, "wrist")}
    private val armExtension by lazy {hardwareMap.get(DcMotorEx::class.java, "extender")}

    private var isGateClosed = true
    private var armTarget: Double = -1.0
    private var spinnerPower: Double = 0.0

    @Config
    object ArmPIDCoefficients {
        @JvmField var ARM_P = 0.0
    }

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
        // Deal with hang mechanism (left bumper to retract, right bumper to extend)
        when {
            gamepad1.left_bumper -> {
                hangMotor1.power = -hangMotorPower
                hangMotor2.power = -hangMotorPower
            }
            gamepad1.right_bumper -> {
                hangMotor1.power = hangMotorPower
                hangMotor2.power = hangMotorPower
            }
            else -> {
                hangMotor1.power = 0.0
                hangMotor2.power = 0.0
            }
        }
        // Deal with drivetrain
        curvatureDrive(
                squareWithSign(-gamepad1.left_stick_y.toDouble()),
                squareWithSign(gamepad1.right_stick_x.toDouble()),
                gamepad1.right_stick_button
        )

        // Deal with deposit/intake
        when {
            gamepad2.a -> {
                armTarget = 0.0
                spinnerPower = -1.0 // Direction to intake
                isGateClosed = true
                elbow.position = 0.0 // Ground pos
                wrist.position = 0.0 // Ground pos
            }
            gamepad2.b -> {
                armTarget = 90.0
                spinnerPower = -1.0 // Direction to intake
                isGateClosed = true
                elbow.position = 0.0 // Ground pos
                wrist.position = 0.0 // Ground pos
            }
            gamepad2.x -> {
                armTarget = 120.0
                spinnerPower = -1.0 // Direction to intake
                isGateClosed = true
                elbow.position = 1.0 // Lifted pos
                wrist.position = 1.0 // Extended pos
            }
            gamepad2.y -> {
                armTarget = 120.0
                spinnerPower = -1.0 // Direction to intake
                isGateClosed = false
                elbow.position = 1.0 // Lifted pos
                wrist.position = 1.0 // Extended pos
            }
        }

        val armAngle = (armPotent.voltage/armPotent.maxVoltage)*270.0 //Need to add/subtract so that 0 is parallel to floor

        if (armTarget != -1.0) {
            val speed = (armAngle - armTarget) * ArmPIDCoefficients.ARM_P
            leftArmMotor.power = speed
            rightArmMotor.power = speed
        }

        if (!gamepad2.right_bumper) {
            spinner.power = spinnerPower
        }
        else {
            spinner.power = 1.0 // Direction to reverse intake
        }

        gate.position = when (isGateClosed) {
            true -> 1.0 // Closed pos
            false -> 0.0 // Opened pos
        }
        armExtension.power = squareWithSign(gamepad2.left_stick_y.toDouble())

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