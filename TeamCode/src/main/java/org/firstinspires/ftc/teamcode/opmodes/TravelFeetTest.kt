package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.commands.TravelFeet

@Autonomous(name="TravelFeetTest")
class TravelFeetTest : CommandOpMode(TravelFeet(2.0))