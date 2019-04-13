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
import org.firstinspires.ftc.teamcode.constants.DepotAutoConstants
import org.firstinspires.ftc.teamcode.constants.SharedAutoConstants
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

    private val hangMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang")}

    private val markerServo by lazy {hardwareMap!!.get(Servo::class.java, "marker")}

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

        markerServo.position = SharedAutoConstants.MARKER_INIT_POS

        waitForStart()

        val goldPosition = Util.getGoldPosition(this, samplingHelper)

        Util.land(this, hangMotor, hangSensor)

        Util.moveFeet(SharedAutoConstants.GET_OFF_HOOK_DISTANCE,
                SharedAutoConstants.GET_OFF_HOOK_POWER, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)

        Util.sample(this, goldPosition,
                leftFrontMotor, leftBackMotor, leftFrontMotor, rightFrontMotor, imu)

        Util.moveFeet(SharedAutoConstants.HIT_SAMPLE_DISTANCE, SharedAutoConstants.HIT_SAMPLE_POWER,
                this, leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)

        Util.moveFeet(SharedAutoConstants.BACK_FROM_SAMPLE_DISTANCE,
                SharedAutoConstants.BACK_FROM_SAMPLE_POWER, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)

        Util.turnToAngle(AngleUnit.DEGREES, SharedAutoConstants.TURN_TO_WALL_ANGLE, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, imu)

        Util.travelToDistance(SharedAutoConstants.TARGET_DISTANCE_FROM_WALL, this,
                rangeSensor, leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor)

        Util.turnToAngle(AngleUnit.DEGREES, DepotAutoConstants.TURN_TOWARDS_DEPOT_ANGLE,
                this, leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, imu)

        Util.moveFeet(DepotAutoConstants.DRIVE_TO_DEPOT_DISTANCE,
                DepotAutoConstants.DRIVE_TO_DEPOT_POWER, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)

        // Drop
        markerServo.position = SharedAutoConstants.MARKER_DROP_POS

        sleep(300)

        Util.moveFeet(DepotAutoConstants.DRIVE_TO_CRATER_DISTANCE,
                DepotAutoConstants.DRIVE_TO_CRATER_POWER, this,
                leftFrontMotor, leftBackMotor, rightFrontMotor,  rightBackMotor)
    }
}