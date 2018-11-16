package org.firstinspires.ftc.teamcode.robot.subsystems

import com.jdroids.robotlib.command.Command
import com.jdroids.robotlib.command.SchedulerImpl
import com.jdroids.robotlib.command.Subsystem
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.pathplanning.DriveTrainStatistics
import org.firstinspires.ftc.teamcode.pathplanning.MotionProfilingConstraints
import org.firstinspires.ftc.teamcode.pathplanning.MotorVelocity
import kotlin.math.roundToInt

class Drive : Subsystem {
    lateinit var opMode: OpMode

    private val leftMotor
            by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "left")}
    private val rightMotor
            by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "right")}

    val statistics = DriveTrainStatistics(1.0/3.0, 1.145833)
    val constraints = MotionProfilingConstraints(2.3, 0.75)

    private val wheelCircumference = statistics.wheelRadius * Math.PI * 2

    var motorVelocity = MotorVelocity(0.0, 0.0)

    override fun initHardware() {
        rightMotor.direction = DcMotorSimple.Direction.REVERSE

        rightMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        leftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        SchedulerImpl.register(this)
    }

    override fun periodic() {
        leftMotor.setVelocity(
                motorVelocity.leftVelocity / wheelCircumference / Math.PI * 2,
                AngleUnit.RADIANS)
        rightMotor.setVelocity(
                motorVelocity.rightVelocity / wheelCircumference / Math.PI * 2,
                AngleUnit.RADIANS)

        opMode.telemetry.addData("Motor Velocity", motorVelocity)
        opMode.telemetry.addData("Left Power", leftMotor.velocity)
        opMode.telemetry.addData("Right Power", rightMotor.velocity)
    }

    //Util commands

    private fun feetToEncoderTicks(feet: Double) =
            ((1120.0*12.0*feet) / (4 * Math.PI)).roundToInt()

    inner class DriveFeet(private val feet: Double) : Command {
        override fun start() {
            leftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
            rightMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

            leftMotor.targetPosition = feetToEncoderTicks(feet)
            rightMotor.targetPosition = feetToEncoderTicks(feet)

            leftMotor.power = 0.5
            rightMotor.power = 0.5
        }

        override fun periodic() {}

        override fun end() {
            leftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            rightMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }

        override fun interrupt() {
            SchedulerImpl.clearSubsystemRequirements(this)
            end()
        }

        override fun isCompleted() = leftMotor.isBusy || rightMotor.isBusy

        override fun isInterruptible() = true
    }
}