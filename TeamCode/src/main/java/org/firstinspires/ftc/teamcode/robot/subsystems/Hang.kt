package org.firstinspires.ftc.teamcode.robot.subsystems

import com.jdroids.robotlib.command.SchedulerImpl
import com.jdroids.robotlib.command.Subsystem
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo

class Hang : Subsystem {
    init {
        SchedulerImpl.register(this)
    }

    lateinit var opMode: OpMode

    private val hangMotor1 by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "hang1")}
    private val hangMotor2 by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "hang2")}
    private val hangServo by lazy {opMode.hardwareMap.get(Servo::class.java, "hangServo")}

    enum class HookState {
        LATCHED,
        OPENED,
        NEUTRAL
    }

    enum class LiftState{
        UP,
        DOWN,
        NEUTRAL
    }

    var hookState = HookState.NEUTRAL
    var liftState = LiftState.NEUTRAL

    override fun initHardware() {
        hangMotor1.direction = DcMotorSimple.Direction.REVERSE

        hangMotor2
        hangServo
    }

    override fun periodic() {
        when (hookState) {
            HookState.LATCHED -> hangServo.position = 0.0
            HookState.OPENED -> hangServo.position = 1.0
            else -> {}
        }

        val power = when (liftState) {
            LiftState.UP -> 0.8
            LiftState.DOWN -> -0.8
            else -> 0.0
        }

        hangMotor1.power = power
        hangMotor2.power = power
    }
}