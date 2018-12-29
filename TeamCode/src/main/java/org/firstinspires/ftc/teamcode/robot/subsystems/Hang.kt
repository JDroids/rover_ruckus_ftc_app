package org.firstinspires.ftc.teamcode.robot.subsystems

import com.jdroids.robotlib.command.SchedulerImpl
import com.jdroids.robotlib.command.Subsystem
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

class Hang : Subsystem {
    lateinit var opMode: OpMode

    private val hangMotor1 by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "hang1")}
    private val hangMotor2 by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "hang2")}

    enum class State {
        UP,
        DOWN,
        NEUTRAL
    }

    var state = State.NEUTRAL

    private val liftSpeed = 0.9

    override fun initHardware() {
        hangMotor1.direction = DcMotorSimple.Direction.REVERSE
        hangMotor2 // This initializes the "by lazy" element for hangMotor2

        SchedulerImpl.register(this)
    }

    override fun periodic() {
        val power = when (state) {
            State.UP -> liftSpeed
            State.DOWN -> -liftSpeed
            State.NEUTRAL -> 0.0
        }

        hangMotor1.power = power
        hangMotor2.power = power
    }
}