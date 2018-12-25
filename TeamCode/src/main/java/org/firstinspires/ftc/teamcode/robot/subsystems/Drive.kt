package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.drive.TankDrive
import com.jdroids.robotlib.command.SchedulerImpl
import com.jdroids.robotlib.command.Subsystem
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Util
import org.firstinspires.ftc.teamcode.pathplanning.DriveTrainStatistics
import org.firstinspires.ftc.teamcode.pathplanning.MotionProfilingConstraints
import org.firstinspires.ftc.teamcode.pathplanning.MotorVelocity
import org.firstinspires.ftc.teamcode.Util.getRadians

class Drive : Subsystem {
    var initialPose = Pose2d()

    lateinit var opMode: OpMode

    private val leftFrontMotor
            by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "lf")}
    private val leftBackMotor
            by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "lb")}

    private val rightFrontMotor
            by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "rf")}
    private val rightBackMotor
            by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "rb")}

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

    inner class DriveWrapper : TankDrive(statistics.wheelDistance) {
        override fun setMotorPowers(left: Double, right: Double) {
            motorVelocity = MotorVelocity(
                    right * constraints.maximumVelocity,
                    left * constraints.maximumVelocity
            )
        }

        override fun getWheelPositions(): List<Double> {
            val leftPosition =
                    (leftFrontMotor.currentPosition + leftBackMotor.currentPosition) / 2
        val rightPosition =
                    (rightFrontMotor.currentPosition + rightBackMotor.currentPosition) / 2

            return listOf(leftPosition.toDouble(), rightPosition.toDouble())
        }

        override fun getExternalHeading(): Double = imu.getRadians() + initialPose.heading
    }
}
