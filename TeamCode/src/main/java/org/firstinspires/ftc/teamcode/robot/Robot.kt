package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive
import org.firstinspires.ftc.teamcode.robot.subsystems.Hang
import kotlin.properties.Delegates

object Robot {
    val drive = Drive()
    val hang = Hang()

    fun initHardware(opMode: OpMode) {
        drive.opMode = opMode
        hang.opMode = opMode

        drive.initHardware()
        // hang.initHardware()
    }
}