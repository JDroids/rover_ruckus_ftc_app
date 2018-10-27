package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Util
import org.firstinspires.ftc.teamcode.Util.toRadians

@Autonomous(name="TurnTest")
class TurnTest : LinearOpMode() {
    private val leftMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "left")}
    private val rightMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "right")}
    private val imu by lazy {hardwareMap!!.get(BNO055IMU::class.java, "imu")}

    override fun runOpMode() {
        leftMotor.direction = DcMotorSimple.Direction.REVERSE

        leftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        Util.initializeIMU(imu)

        waitForStart()

        Util.turnToRelativeAngle(90.toRadians(), this, leftMotor, rightMotor, imu)
    }


}