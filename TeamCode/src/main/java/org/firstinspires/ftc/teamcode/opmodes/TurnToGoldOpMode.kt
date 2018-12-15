package org.firstinspires.ftc.teamcode.opmodes

import com.jdroids.robotlib.command.SchedulerImpl
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.Util
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.commands.TurnToGold

@Autonomous(name="TurnToGold")
class TurnToGoldOpMode : LinearOpMode() {
    private val leftMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "left")}
    private val rightMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "right")}

    override fun runOpMode() {
        Robot.initHardware(this)

        val turnToGold = TurnToGold(this)

        SchedulerImpl.run(turnToGold)

        waitForStart()

        while (!turnToGold.isCompleted() && opModeIsActive()) {
            SchedulerImpl.periodic()

            telemetry.update()
        }

        leftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        Util.moveFeet(2.0, this, leftMotor, rightMotor)
    }
}