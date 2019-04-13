package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.TeleOpConstants

@TeleOp(name="One Person TeleOp")
class OnePersonTeleOp : OpMode() {
    private val hangMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang")}

    private val leftFrontMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "lf")}
    private val leftBackMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "lb")}

    private val rightFrontMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "rf")}
    private val rightBackMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "rb")}

    private val leftArmMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "leftArmMotor")}
    private val rightArmMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "rightArmMotor")}

    private val pot by lazy {hardwareMap.get(AnalogInput::class.java, "pot")}
    private val spinner by lazy {hardwareMap.get(DcMotor::class.java, "intake")}
    private val gate by lazy {hardwareMap.get(Servo::class.java, "gate")}
    private val elbow by lazy {hardwareMap.get(Servo::class.java, "elbow")}
    private val wrist by lazy {hardwareMap.get(Servo::class.java, "wrist")}
    private val armExtension by lazy {hardwareMap.get(CRServo::class.java, "extender")}

    private var armTarget: Double = -1.0
    private var spinnerPower: Double = 0.0

    override fun init() {
        rightFrontMotor.direction = DcMotorSimple.Direction.REVERSE
        rightBackMotor.direction = DcMotorSimple.Direction.REVERSE
    }

    private val hangMotorPower = 0.9
    private val armMotorPower = 0.2

    private var previousVoltage = 0.0
    private val timer = ElapsedTime()

    override fun start() {
        elbow.position = TeleOpConstants.ELBOW_POSITION
        previousVoltage = pot.voltage
    }

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
                squareWithSign(-gamepad1.right_stick_x.toDouble()),
                gamepad1.right_stick_button
        )


        // Deal with deposit/intake
        when {
            // Ground
            gamepad1.a -> {
                spinnerPower = -0.9 // Direction to intake
                wrist.position = TeleOpConstants.WRIST_INTAKE // Ground pos
                gate.position = TeleOpConstants.GATE_CLOSED_POS
            }

            // Deposit
            gamepad1.b -> {
                spinnerPower = -0.9
                wrist.position = TeleOpConstants.WRIST_DEPOSIT
                gate.position = TeleOpConstants.GATE_CLOSED_POS
            }

            // Gold gate
            gamepad1.x -> {
                gate.position = TeleOpConstants.GATE_GOLD_POS
            }
            // Silver gate
            gamepad1.y -> {
                gate.position = TeleOpConstants.GATE_SILVER_POS
            }
        }

        if (gamepad1.dpad_right) {
            spinnerPower = 0.0
        }
        else if (gamepad1.dpad_left) {
            spinnerPower = 0.9
        }

        spinner.power = spinnerPower

        val armRotPower =
                if (Math.abs(gamepad1.left_trigger) < 0.04 && Math.abs(gamepad1.right_trigger) < 0.04) {
                    (pot.voltage - previousVoltage) * -25
                }
                else {
                    previousVoltage = pot.getVoltage()
                    if (gamepad1.left_trigger > 0.004) {
                        gamepad1.left_trigger.toDouble()
                    }
                    else {
                        -gamepad1.right_trigger.toDouble()
                    }
                }

        leftArmMotor.power = armRotPower
        rightArmMotor.power = -armRotPower

        previousVoltage = pot.voltage
        timer.reset()

        armExtension.power = when {
            gamepad1.dpad_up -> 0.9
            gamepad1.dpad_down -> -0.9
            else -> 0.0
        }


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