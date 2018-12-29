package org.firstinspires.ftc.teamcode.robot.commands

import com.jdroids.robotlib.command.Command
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.Hang

class Land : Command {
    override fun start() {
        Robot.hang.state = Hang.State.UP
    }

    override fun periodic() {}

    override fun end() {
        Robot.hang.state = Hang.State.NEUTRAL
    }

    override fun isCompleted() = Robot.hang.isSensorActive
}