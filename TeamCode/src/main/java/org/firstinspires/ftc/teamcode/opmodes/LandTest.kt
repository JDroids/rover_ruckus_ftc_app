package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.Util

@Autonomous(name="LandTest")
class LandTest : LinearOpMode() {
    private val hangMotor1 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang1")}
    private val hangMotor2 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang2")}

    private val hangSensor by lazy {
        hardwareMap!!.get(DigitalChannel::class.java, "hangSensor")}

    override fun runOpMode() {
        hangMotor1.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        Util.land(this, hangMotor1, hangMotor2, hangSensor)
    }
}