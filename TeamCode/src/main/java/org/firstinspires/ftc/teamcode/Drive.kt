package org.firstinspires.ftc.teamcode

import com.jdroids.robotlib.command.Subsystem
import com.jdroids.robotlib.util.getActiveOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class Drive : Subsystem() {
    private val leftMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "left")}
    private val rightMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "left")}

    var leftPower = 0.0
    var rightPower = 0.0

    override fun initDefaultCommand() {}

    override fun initHardware(hardwareMap: HardwareMap) {
        rightMotor.direction = DcMotorSimple.Direction.REVERSE

        // rightMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        // leftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        super.initHardware(hardwareMap)
    }

    override fun periodic() {
        leftMotor.power = leftPower
        rightMotor.power = rightPower

        getActiveOpMode().telemetry.addData("Left Target Power", leftPower)
        getActiveOpMode().telemetry.addData("Right Target Power", rightPower)
        getActiveOpMode().telemetry.addData("Left Power", leftMotor.power)
        getActiveOpMode().telemetry.addData("Right Power", rightMotor.power)
    }
}