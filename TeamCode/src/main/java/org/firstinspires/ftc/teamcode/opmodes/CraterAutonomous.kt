package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.*
//import org.firstinspires.ftc.teamcode.SamplingVision
import org.firstinspires.ftc.teamcode.Util
import org.firstinspires.ftc.teamcode.Util.toRadians
import org.firstinspires.ftc.teamcode.pathplanning.*

@Autonomous(name="Crater Auto")
class CraterAutonomous : LinearOpMode() {
    private val leftMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "left")}
    private val rightMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "right")}
    private val imu by lazy {hardwareMap!!.get(BNO055IMU::class.java, "imu")}

    private val hangMotor1 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang1")}
    private val hangMotor2 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang2")}
    private val hangServo by lazy {hardwareMap!!.get(Servo::class.java, "hangServo")}

    private val markerServo by lazy {hardwareMap!!.get(Servo::class.java, "depotServo")}

    private val hangTOFSensor by lazy {
        hardwareMap!!.get(Rev2mDistanceSensor::class.java, "tofSensor")}

    private val frontUltrasonic
            by lazy {hardwareMap!!.get(ModernRoboticsI2cRangeSensor::class.java, "rangeFront")}

    override fun runOpMode() {
        leftMotor.direction = DcMotorSimple.Direction.REVERSE

        leftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        hangMotor2.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        Util.land(this, hangMotor1, hangMotor2, hangServo, hangTOFSensor)

        leftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        leftMotor.power = -1.0
        rightMotor.power = -1.0

        sleep(300)

        leftMotor.power = 0.0
        rightMotor.power = 0.0

        sleep(100)

        Util.setHookState(Util.HookState.OPENED, hangServo)

        sleep(200)

        Util.initializeIMU(imu)

        Util.moveFeet(2.5, this, leftMotor, rightMotor)

        sleep(100)

        Util.turnTime(1500, -0.3, this, leftMotor, rightMotor)

        Util.moveFeet(3.3, this, leftMotor, rightMotor)

        Util.turnTime(800, -0.3, this, leftMotor, rightMotor)

        Util.moveFeet(3.9, this, leftMotor, rightMotor)

        Util.turnTime(600, 0.3, this, leftMotor, rightMotor)

        Util.setMarkerState(Util.MarkerState.OPENED, markerServo)

        Util.turnTime(500, -0.3, this, leftMotor, rightMotor)

        Util.moveFeet(-8.5, this, leftMotor, rightMotor)

        sleep(100)

    }
}