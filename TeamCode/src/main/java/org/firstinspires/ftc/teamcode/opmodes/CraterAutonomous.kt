package org.firstinspires.ftc.teamcode.opmodes

import com.jdroids.robotlib.command.Command
import com.jdroids.robotlib.command.SchedulerImpl
import com.jdroids.robotlib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.commands.Land
import org.firstinspires.ftc.teamcode.robot.commands.TurnToAngle
import org.firstinspires.ftc.teamcode.robot.commands.TurnToGold
@Disabled
@Autonomous(name="Crater Auto Old 2")
class CraterAutonomous : LinearOpMode() {
    override fun runOpMode() {
        Robot.initHardware(this)

        waitForStart()

        runCommand(Land())
        runCommand(Robot.drive.travelFeet(0.1))
        runCommand(TurnToGold())
        runCommand(Robot.drive.travelFeet(3.2))
        runCommand(Robot.drive.travelFeet(-1.8))
        runCommand(TurnToAngle(AngleUnit.DEGREES, 110.0))
        runCommand(Robot.drive.travelFeet(3.0))
        runCommand(TurnToAngle(AngleUnit.DEGREES,45.0))
        runCommand(Robot.drive.travelFeet(5.0))
        runCommand(Robot.drive.travelFeet(-6.0))

        SchedulerImpl.kill()
    }

    private fun runCommand(command: Command) {
        SchedulerImpl.run(command)

        while (!command.isCompleted() && opModeIsActive()) {
            SchedulerImpl.periodic()
            telemetry.update()
        }
    }
}