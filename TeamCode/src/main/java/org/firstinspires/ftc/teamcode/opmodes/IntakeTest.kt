/*package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name="Intake Positions ")
class IntakeTest : OpMode() {

    private val armPotent by lazy {hardwareMap.get(AnalogInput::class.java, "potent")}
    private val gate by lazy {hardwareMap.get(Servo::class.java, "gate")}
    private val elbow by lazy {hardwareMap.get(Servo::class.java, "elbow")}
    private val wrist by lazy {hardwareMap.get(Servo::class.java, "wrist")}

    override fun init() {
        gate.position = 0.0
    }

    override fun loop() {
        telemetry.addData("Potentiometer", armPotent.voltage/armPotent.maxVoltage*270)
        telemetry.addData("Gate pos", gate.position)
        if (gamepad1.a) {
            gate.position = 1.0
        }
        telemetry.addData("Elbow pos", elbow.position)
        telemetry.addData("Wrist pos", wrist.position)
        telemetry.update()
    }
}*/