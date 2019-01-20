package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive
import org.firstinspires.ftc.teamcode.robot.subsystems.Hang
import kotlin.properties.Delegates

object Robot {
    lateinit var hub: LynxModule
    val getHub = {opMode.hardwareMap.get(LynxModule::class.java, "Expansion Hub 2")}

    val drive = Drive()
    val hang = Hang()

    lateinit var opMode: OpMode
        private set

    fun initHardware(opMode: OpMode) {
        this.opMode = opMode

        drive.opMode = opMode
        // hang.opMode = opMode

        drive.initHardware()
        //hang.initHardware()
    }
}