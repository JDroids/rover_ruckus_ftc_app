package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Util
import org.firstinspires.ftc.teamcode.robot.SamplingHelper

@Autonomous(name="Crater Auto")
class CraterAuto : LinearOpMode() {
    private val leftFrontMotor
            by lazy {hardwareMap.get(DcMotorEx::class.java, "lf")}
    private val leftBackMotor
            by lazy {hardwareMap.get(DcMotorEx::class.java, "lb")}

    private val rightFrontMotor
            by lazy {hardwareMap.get(DcMotorEx::class.java, "rf")}
    private val rightBackMotor
            by lazy {hardwareMap.get(DcMotorEx::class.java, "rb")}

    private val hangMotor1 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang1")}
    private val hangMotor2 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang2")}

    private val markerServo by lazy {hardwareMap!!.get(Servo::class.java, "depotServo")}

    private val hangSensor by lazy {
        hardwareMap!!.get(DigitalChannel::class.java, "hangSensor")}

    private val imu by lazy {hardwareMap!!.get(BNO055IMU::class.java, "imu")}

    override fun runOpMode() {
        rightFrontMotor.direction = DcMotorSimple.Direction.REVERSE
        rightBackMotor.direction = DcMotorSimple.Direction.REVERSE

        val samplingHelper = SamplingHelper(this)

        Util.initializeIMU(imu)

        waitForStart()

        //Util.land(this, hangMotor1, hangMotor2, hangSensor)


        Util.moveFeet(0.3, 0.3, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)

        Util.turnToGold(this, samplingHelper, leftFrontMotor, leftBackMotor,
                rightFrontMotor, rightBackMotor)

        Util.moveFeet(2.8, 0.3, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)

        Util.moveFeet(-2.4, 0.5, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)

        Util.turnToAngle(AngleUnit.DEGREES, 180.0, this, leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, imu)

        Util.moveFeet(0.6, 0.5, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)

        Util.turnToAngle(AngleUnit.DEGREES, 265.0, this, leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, imu)

        Util.moveFeet(2.5, 0.5, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)

        Util.turnToAngle(AngleUnit.DEGREES, 315.0, this, leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, imu)

        Util.moveFeet(4.0, 0.5, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)

        // deposit marker here

        Util.turnToAngle(AngleUnit.DEGREES, 320.0, this, leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, imu)

        Util.moveFeet(-5.0, 0.3, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)
    }
}