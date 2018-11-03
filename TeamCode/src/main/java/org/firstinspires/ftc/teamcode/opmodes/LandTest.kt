package org.firstinspires.ftc.teamcode.opmodes

//import com.jdroids.robotlib.util.getNameOfHardwareDevice
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Util
/*
@Disabled
@Autonomous(name="LandTest")
class LandTest : LinearOpMode() {
    private val hangMotor1 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang1")}
    private val hangMotor2 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang2")}

    private val hangServo by lazy {hardwareMap!!.get(Servo::class.java, "hangServo")}
    private val hangTOFSensor by lazy {
        hardwareMap!!.get(Rev2mDistanceSensor::class.java, "tofSensor")}

    override fun runOpMode() {
        hangMotor2.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        Util.land(this, hangMotor1, hangMotor2, hangServo, hangTOFSensor)
    }
}*/