package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.drive.TankDrive
import com.acmerobotics.roadrunner.profile.MotionState
import com.jdroids.robotlib.command.SchedulerImpl
import com.jdroids.robotlib.command.Subsystem
import com.jdroids.robotlib.command.command
import com.jdroids.robotlib.controller.PIDControllerImpl
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Util
import org.firstinspires.ftc.teamcode.Util.encoderTicksToFeet
import org.firstinspires.ftc.teamcode.Util.feetToEncoderTicks
import org.firstinspires.ftc.teamcode.pathplanning.DriveTrainStatistics
import org.firstinspires.ftc.teamcode.pathplanning.MotionProfilingConstraints
import org.firstinspires.ftc.teamcode.pathplanning.MotorVelocity
import org.firstinspires.ftc.teamcode.Util.getRadians

class Drive : Subsystem {
    lateinit var opMode: OpMode

    private val leftFrontMotor
            by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "lf")}
    private val leftBackMotor
            by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "lb")}

    private val rightFrontMotor
            by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "rf")}
    private val rightBackMotor
            by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "rb")}

    private val leftMotors by lazy { listOf(leftFrontMotor, leftBackMotor)}
    private val rightMotors by lazy {listOf(rightFrontMotor, rightBackMotor)}
    private val motors by lazy {listOf(leftFrontMotor, leftBackMotor,
            rightFrontMotor, rightBackMotor)}

    val imu: BNO055IMU
        by lazy {opMode.hardwareMap.get(BNO055IMU::class.java, "imu")}

    val statistics = DriveTrainStatistics(1.0/3.0, 1.145833)
    val constraints = MotionProfilingConstraints(4.6, 0.75)

    private val wheelCircumference = statistics.wheelRadius * Math.PI * 2

    var motorVelocity = MotorVelocity(0.0, 0.0)

    override fun initHardware() {
        rightFrontMotor.direction = DcMotorSimple.Direction.REVERSE
        rightBackMotor.direction = DcMotorSimple.Direction.REVERSE

        leftFrontMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftBackMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightFrontMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightBackMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        leftFrontMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        leftBackMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightFrontMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightBackMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        Util.initializeIMU(imu)

        SchedulerImpl.register(this)
    }

    override fun periodic() {
        val leftMotorVelocity = motorVelocity.leftVelocity / wheelCircumference / Math.PI * 2
        val rightMotorVelocity = motorVelocity.rightVelocity / wheelCircumference / Math.PI * 2

        leftFrontMotor.setVelocity(leftMotorVelocity, AngleUnit.RADIANS)
        leftBackMotor.setVelocity(leftMotorVelocity, AngleUnit.RADIANS)

        rightFrontMotor.setVelocity(rightMotorVelocity, AngleUnit.RADIANS)
        rightFrontMotor.setVelocity(rightMotorVelocity, AngleUnit.RADIANS)

        opMode.telemetry.addData("Motor Velocity", motorVelocity)
    }

    fun travelFeet(feet: Double) = command {

        val ticks = Util.feetToEncoderTicks(feet)

        start {
            motors.forEach {
                it.apply {
                    mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                    mode = DcMotor.RunMode.RUN_TO_POSITION
                }
            }

            leftMotors.forEach {
                it.apply {
                    targetPosition = ticks
                }
            }

            rightMotors.forEach {
                it.apply {
                    targetPosition = -ticks
                }
            }

            motorVelocity = MotorVelocity(100.0, 100.0)
        }

        end {
            motorVelocity = MotorVelocity(0.0, 0.0)

            motors.forEach {
                it.apply {
                    mode = DcMotor.RunMode.RUN_USING_ENCODER
                }
            }
        }

        isCompleted {
            if (leftFrontMotor.mode == DcMotor.RunMode.RUN_TO_POSITION) {
                motors.all{!it.isBusy}
            }
            else {
                true
            }
        }
    }
}
