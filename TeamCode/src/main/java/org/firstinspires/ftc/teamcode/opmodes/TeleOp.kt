package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.TeleOpConstants

@TeleOp(name="TeleOp")
class TeleOp : OpMode() {
    private val hangMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang")}

    private val leftFrontMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "lf")}
    private val leftBackMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "lb")}

    private val rightFrontMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "rf")}
    private val rightBackMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "rb")}

    private val leftArmMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "leftArmMotor")}
    private val rightArmMotor by lazy {hardwareMap.get(DcMotorEx::class.java, "rightArmMotor")}

    private val pot by lazy {hardwareMap.get(AnalogInput::class.java, "pot")}

    private val intakeClamp by lazy {hardwareMap.get(Servo::class.java, "intake")}
    private val elbow by lazy {hardwareMap.get(Servo::class.java, "elbow")}
    private val armExtension by lazy {hardwareMap.get(CRServo::class.java, "extender")}

    override fun init() {
        rightFrontMotor.direction = DcMotorSimple.Direction.REVERSE
        rightBackMotor.direction = DcMotorSimple.Direction.REVERSE
    }

    private val hangMotorPower = -0.9
    private val armMotorPower = 0.2

    private var previousVoltage = 0.0
    private val timer = ElapsedTime()

    override fun start() {
        //elbow.position = 0.1
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
            // Ground, Open
            gamepad2.a -> {
                elbow.position = TeleOpConstants.ELBOW_INTAKE // Ground pos
                intakeClamp.position = TeleOpConstants.INTAKE_CLAMP_OPEN_POS
            }

            // Ground, Closed
            gamepad2.b -> {
                elbow.position = TeleOpConstants.ELBOW_INTAKE
                intakeClamp.position = TeleOpConstants.INTAKE_CLAMP_CLOSED_POS
            }

            // Deposit, Closed
            gamepad2.y -> {
                elbow.position = TeleOpConstants.ELBOW_DEPOSIT
                intakeClamp.position = TeleOpConstants.INTAKE_CLAMP_CLOSED_POS
            }

            // Deposit, Open
            gamepad2.x -> {
                elbow.position = TeleOpConstants.ELBOW_DEPOSIT
                intakeClamp.position = TeleOpConstants.INTAKE_CLAMP_OPEN_POS
            }

            // Hang position
            gamepad2.left_bumper || gamepad2.right_bumper -> {
                elbow.position = TeleOpConstants.ELBOW_HANG
                intakeClamp.position = TeleOpConstants.INTAKE_CLAMP_CLOSED_POS
            }
        }
        val armRotPower =
                if (Math.abs(gamepad2.left_stick_y) < 0.04) {
                    (pot.voltage - previousVoltage) * -25
                }
                else {
                    previousVoltage = pot.getVoltage()
                    squareWithSign(gamepad2.left_stick_y.toDouble())
                }

        leftArmMotor.power = armRotPower
        rightArmMotor.power = -armRotPower

        previousVoltage = pot.voltage
        timer.reset()

        armExtension.power = squareWithSign(gamepad2.right_stick_y.toDouble())

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