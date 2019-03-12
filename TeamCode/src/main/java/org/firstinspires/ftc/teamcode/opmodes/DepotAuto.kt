package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Util
import org.firstinspires.ftc.teamcode.Util.getRadians
import org.firstinspires.ftc.teamcode.robot.SamplingHelper

@Autonomous(name="Depot Auto")
class DepotAuto : LinearOpMode() {
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

    private val rangeSensor
            by lazy {hardwareMap.get(ModernRoboticsI2cRangeSensor::class.java, "rangeSensor")}

    private val imu by lazy {hardwareMap!!.get(BNO055IMU::class.java, "imu")}

    override fun runOpMode() {
        rightFrontMotor.direction = DcMotorSimple.Direction.REVERSE
        rightBackMotor.direction = DcMotorSimple.Direction.REVERSE

        val samplingHelper = SamplingHelper(this)

        Util.initializeIMU(imu)

        markerServo.position = 0.65

        waitForStart()

        val goldPosition = Util.getGoldPosition(this, samplingHelper)

        Util.land(this, hangMotor1, hangMotor2, hangSensor)

        Util.moveFeet(0.3, 0.3, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)

        Util.sample(this, goldPosition,
                leftFrontMotor, leftBackMotor, leftFrontMotor, rightFrontMotor, imu)

        Util.moveFeet(2.4, 0.3, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)

        Util.moveFeet(-1.9, 0.5, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)

        Util.turnToAngle(AngleUnit.DEGREES, 240.0, this, leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, imu)

        Util.travelToDistance(5.0, this, rangeSensor,
                leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor)


        Util.turnToAngle(AngleUnit.DEGREES, 135.0, this, leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, imu)

        Util.moveFeet(1.4, 0.2, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)

        // Drop
        markerServo.position = 0.0

        sleep(300)

        Util.moveFeet(-3.4, 1.0, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)


        /*
        Util.moveFeet(-2.1, 0.5, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)


        Util.travelToDistance(5.0, this, rangeSensor,
                leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor)


        Util.turnToAngle(AngleUnit.DEGREES, 305.0, this, leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, imu)

        Util.moveFeet(3.5, 0.6, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)

        //Deposit here idiot

        Util.turnToAngle(AngleUnit.DEGREES, 320.0, this, leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, imu)

        Util.moveFeet(-4.6, 0.3, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)


        /*Util.moveFeet(3.0, 0.9, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)


        Util.moveFeet(-6.2, 0.9, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)
        */*/
    }
}