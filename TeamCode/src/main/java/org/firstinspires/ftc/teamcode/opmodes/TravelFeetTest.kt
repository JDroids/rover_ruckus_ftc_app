package org.firstinspires.ftc.teamcode.opmodes

import com.jdroids.robotlib.command.Command
import com.jdroids.robotlib.command.SchedulerImpl
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.Robot

@Disabled
@Autonomous(name="TravelFeetTest")
class TravelFeetTest : LinearOpMode() {
    override fun runOpMode() {
        Robot.initHardware(this)

        waitForStart()

        runCommand(Robot.drive.travelFeet(2.0))
    }

    private fun runCommand(command: Command) {
        SchedulerImpl.run(command)

        while (!command.isCompleted() && opModeIsActive()) {
            SchedulerImpl.periodic()
            telemetry.update()
        }
    }
}