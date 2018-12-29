package org.firstinspires.ftc.teamcode.opmodes

import com.jdroids.robotlib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.robot.commands.Land
import org.firstinspires.ftc.teamcode.robot.commands.TravelFeet
import org.firstinspires.ftc.teamcode.robot.commands.TurnToAngle
import org.firstinspires.ftc.teamcode.robot.commands.TurnToGold

@Autonomous(name="Crater Auto")
class CraterAutonomous : CommandOpMode(
    SequentialCommandGroup(
        Land(),
        TravelFeet(0.1),
        TurnToGold(),
        TravelFeet(3.2),
        TravelFeet(-1.8),
        TurnToAngle(AngleUnit.DEGREES, 135.0),
        TravelFeet(3.0),
        TurnToAngle(AngleUnit.DEGREES,90.0),
        TravelFeet(5.0),
        TravelFeet(-6.0)
    )
)