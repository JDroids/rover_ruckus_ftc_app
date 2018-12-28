package org.firstinspires.ftc.teamcode.opmodes

import com.jdroids.robotlib.command.SchedulerImpl
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.commands.TurnToAngle
import org.firstinspires.ftc.teamcode.Util.toRadians

@Autonomous(name="TurnTest")
class TurnTest : LinearOpMode() {
    val command = TurnToAngle(90.toRadians())

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