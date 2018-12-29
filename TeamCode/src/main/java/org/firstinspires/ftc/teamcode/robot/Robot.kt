package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive
import org.firstinspires.ftc.teamcode.robot.subsystems.Hang
import kotlin.properties.Delegates

object Robot {
    val drive = Drive()
    val hang = Hang()

    lateinit var opMode: OpMode
        private set

    fun initHardware(opMode: OpMode) {
        this.opMode = opMode

        drive.opMode = opMode
        hang.opMode = opMode

        drive.initHardware()
        hang.initHardware()
    }
}