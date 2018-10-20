package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.SamplingVision
import org.firstinspires.ftc.teamcode.Util
import org.firstinspires.ftc.teamcode.Util.toRadians
import org.firstinspires.ftc.teamcode.pathplanning.*

@Autonomous(name="CraterAuto")
class CraterAutonomous : LinearOpMode() {
    private val leftMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "left")}
    private val rightMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "right")}
    private val imu by lazy {hardwareMap!!.get(BNO055IMU::class.java, "imu")}

    private val hangMotor1 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang1")}
    private val hangMotor2 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang2")}
    private val hangServo by lazy {hardwareMap!!.get(Servo::class.java, "hangServo")}

    private val hangTOFSensor by lazy {
        hardwareMap!!.get(Rev2mDistanceSensor::class.java, "tofSensor")}

    override fun runOpMode() {
        leftMotor.direction = DcMotorSimple.Direction.REVERSE

        leftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        hangMotor2.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        Util.land(this, hangMotor1, hangMotor2, hangServo, hangTOFSensor)

        leftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        Util.initializeIMU(imu)

        val goldPosition = SamplingVision.GoldPosition.CENTER //Util.doVision(hardwareMap)

        Util.followPath(LinearPath(Waypoint(0.0, 0.0), Waypoint(0.0, 1.5)),
                this, leftMotor, rightMotor)

        //Util.hitSample(goldPosition, this, leftMotor, rightMotor, imu)
        /*
        Util.followPath(LinearPath(Waypoint(0.0, 0.0), Waypoint(0.0, 0.25)),
                this, leftMotor, rightMotor, true)

        Util.turnToAngle(80.toRadians(), this, leftMotor, rightMotor, imu)

        Util.followPath(LinearPath(Waypoint(0.0, 0.0), Waypoint(0.0, 1.5)),
                this, leftMotor, rightMotor)

        Util.turnToAngle(135.toRadians(), this, leftMotor, rightMotor, imu)

        Util.followPath(LinearPath(Waypoint(0.0, 0.0), Waypoint(0.0, 6.25)),
                this, leftMotor, rightMotor)

        //Deposit marker here

        Util.followPath(LinearPath(Waypoint(0.0, 0.0), Waypoint(0.0, 6.5)),
                this, leftMotor, rightMotor, true)*/
    }
}