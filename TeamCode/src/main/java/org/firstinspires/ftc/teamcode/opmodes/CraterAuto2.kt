package org.firstinspires.ftc.teamcode.opmodes

import com.jdroids.robotlib.command.SchedulerImpl
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.pathplanning.LinearPath
import org.firstinspires.ftc.teamcode.pathplanning.Waypoint
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.commands.FollowConstantCurvaturePath
import org.firstinspires.ftc.teamcode.robot.commands.TurnToGold
@Disabled
@Autonomous(name="CraterAuto2")
class CraterAuto2 : LinearOpMode() {
    override fun runOpMode() {
        Robot.initHardware(this)

        val turnToGold = TurnToGold(this)

        SchedulerImpl.run(turnToGold)

        waitForStart()

        while (!turnToGold.isCompleted() && opModeIsActive()) {
            SchedulerImpl.periodic()

            telemetry.update()
        }

        if (turnToGold.isCompleted()) {
            SchedulerImpl.periodic()
        }
        else {
            turnToGold.end()
        }

        SchedulerImpl.clearRequirements(turnToGold)

        val hitGold = FollowConstantCurvaturePath(
                LinearPath(Waypoint(0.0, 0.0), Waypoint(0.0, 2.0)))

        SchedulerImpl.run(hitGold)

        while (!hitGold.isCompleted() && opModeIsActive()) {
            SchedulerImpl.periodic()

            telemetry.update()
        }

        if (hitGold.isCompleted()) {
            SchedulerImpl.periodic()
        }
        else {
            hitGold.end()
        }

        SchedulerImpl.clearRequirements(hitGold)
    }
}