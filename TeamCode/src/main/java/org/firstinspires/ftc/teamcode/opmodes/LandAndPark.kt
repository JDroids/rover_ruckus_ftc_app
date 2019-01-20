package org.firstinspires.ftc.teamcode.opmodes

import com.jdroids.robotlib.command.Command
import com.jdroids.robotlib.command.SchedulerImpl
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.pathplanning.MotorVelocity
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.commands.Land

@Disabled
@Autonomous(name="Land and Park")
class LandAndPark : LinearOpMode() {
    override fun runOpMode() {
        Robot.initHardware(this)

        waitForStart()

        runCommand(Land())

        Robot.drive.motorVelocity = MotorVelocity(3.0, 3.0)

        sleep(3000)

        Robot.drive.motorVelocity = MotorVelocity(0.0, 0.0)
    }

    private fun runCommand(command: Command) {
        SchedulerImpl.run(command)

        while (!command.isCompleted() && opModeIsActive()) {
            SchedulerImpl.periodic()
            telemetry.update()
        }
    }
}