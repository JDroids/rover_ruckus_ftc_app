package org.firstinspires.ftc.teamcode.opmodes

import com.jdroids.robotlib.command.SchedulerImpl
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.commands.TurnToGold

@Autonomous(name="TurnToGold")
class TurnToGoldOpMode : LinearOpMode() {
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
    }
}