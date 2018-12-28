package org.firstinspires.ftc.teamcode.opmodes

import com.jdroids.robotlib.command.Command
import com.jdroids.robotlib.command.SchedulerImpl
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.Robot

abstract class CommandOpMode(private val command: Command) : LinearOpMode() {
    override fun runOpMode() {
        Robot.initHardware(this)

        waitForStart()

        SchedulerImpl.run(command)

        while (!command.isCompleted() && opModeIsActive()) {
            SchedulerImpl.periodic()
            telemetry.update()
        }

        command.end()

        SchedulerImpl.kill()
    }
}