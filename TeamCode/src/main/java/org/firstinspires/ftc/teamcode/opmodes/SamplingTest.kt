package org.firstinspires.ftc.teamcode.opmodes

import com.jdroids.robotlib.command.SchedulerImpl
import com.jdroids.robotlib.command.SequentialCommandGroup
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Util
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.commands.TurnToGold

@Autonomous(name="Sampling Test")
class SamplingTest : LinearOpMode() {
    private val leftMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "left")}
    private val rightMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "right")}
    private val hangMotor1 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang1")}
    private val hangMotor2 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang2")}
    private val hangServo by lazy {hardwareMap!!.get(Servo::class.java, "hangServo")}
    private val hangTOFSensor by lazy {
        hardwareMap!!.get(Rev2mDistanceSensor::class.java, "tofSensor")}


    override fun runOpMode() {
        Robot.initHardware(this)

        waitForStart()

        Util.land(this, hangMotor1, hangMotor2, hangServo, hangTOFSensor)

        val turnToGold = TurnToGold(this)


        SchedulerImpl.run(turnToGold)

        while (!turnToGold.isCompleted() && opModeIsActive()) {
            SchedulerImpl.periodic()

            telemetry.update()
        }

        Util.moveFeet(2.0, this, leftMotor, rightMotor)

        if (!turnToGold.isCompleted()) {
            turnToGold.interrupt()
        }
    }
}