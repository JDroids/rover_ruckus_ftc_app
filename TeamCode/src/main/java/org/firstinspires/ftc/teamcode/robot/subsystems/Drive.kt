package org.firstinspires.ftc.teamcode.robot.subsystems

import com.jdroids.robotlib.command.SchedulerImpl
import com.jdroids.robotlib.command.Subsystem
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.pathplanning.DriveTrainStatistics
import org.firstinspires.ftc.teamcode.pathplanning.MotionProfilingConstraints
import org.firstinspires.ftc.teamcode.pathplanning.MotorVelocity

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

    val statistics = DriveTrainStatistics(1.0/3.0, 1.145833)
    val constraints = MotionProfilingConstraints(4.6, 0.75)

    private val wheelCircumference = statistics.wheelRadius * Math.PI * 2

    var motorVelocity = MotorVelocity(0.0, 0.0)

    override fun initHardware() {
        rightFrontMotor.direction = DcMotorSimple.Direction.REVERSE
        rightBackMotor.direction = DcMotorSimple.Direction.REVERSE

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
}
