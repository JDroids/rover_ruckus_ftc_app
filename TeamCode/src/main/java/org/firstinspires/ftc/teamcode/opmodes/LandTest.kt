package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.Util

@Autonomous(name="LandTest")
class LandTest : LinearOpMode() {
    private val hangMotor1 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang")}

    private val hangSensor by lazy {
        hardwareMap!!.get(DigitalChannel::class.java, "hangSensor")}

    override fun runOpMode() {
        waitForStart()

        Util.land(this, hangMotor1, hangSensor)
    }
}