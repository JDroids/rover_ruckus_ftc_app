package org.firstinspires.ftc.teamcode.opmodes

import com.jdroids.robotlib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.commands.Land
import org.firstinspires.ftc.teamcode.robot.commands.TurnToAngle
import org.firstinspires.ftc.teamcode.robot.commands.TurnToGold

@Disabled
@Autonomous(name="Depot Auto Old")
class DepotAutonomous : CommandOpMode(
    SequentialCommandGroup(
        Land(),
        Robot.drive.travelFeet(0.1),
        TurnToGold(),
        Robot.drive.travelFeet(3.8),
        Robot.drive.travelFeet(-1.8),
        TurnToAngle(AngleUnit.DEGREES, 110.0),
        Robot.drive.travelFeet(3.0),
        TurnToAngle(AngleUnit.DEGREES,45.0),
        Robot.drive.travelFeet(5.0)
    )
)