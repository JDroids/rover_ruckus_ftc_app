package org.firstinspires.ftc.teamcode.opmodes

import com.jdroids.robotlib.command.Command
import com.jdroids.robotlib.command.SchedulerImpl
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Util
import org.firstinspires.ftc.teamcode.Util.getRadians
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.commands.TurnToGold

@Disabled
@Autonomous(name="Depot Auto Old")
class DepotAutoOld : LinearOpMode() {
    private val leftMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "left")}
    private val rightMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "right")}

    private val hangMotor1 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang1")}
    private val hangMotor2 by lazy {hardwareMap!!.get(DcMotorEx::class.java, "hang2")}

    private val markerServo by lazy {hardwareMap!!.get(Servo::class.java, "depotServo")}

    private val hangSensor by lazy {
        hardwareMap!!.get(DigitalChannel::class.java, "hangSensor")}

    private val imu by lazy {hardwareMap!!.get(BNO055IMU::class.java, "imu")}

    override fun runOpMode() {
        Robot.initHardware(this)

        val garbage = markerServo.position

        Util.initializeIMU(imu)

        waitForStart()

        Util.land(this, hangMotor1, hangMotor2, hangSensor)

        Util.moveFeet(-0.1, this, leftMotor, rightMotor)

        val turnToGold = TurnToGold()

        SchedulerImpl.run(turnToGold)

        updateCommandState(turnToGold)

        val angle = imu.getRadians()

        leftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        Util.moveFeet(-3.2, this, leftMotor, rightMotor)

        turnToAngle(-Math.PI)

        Util.moveFeet(-1.0, this, leftMotor, rightMotor)

        Util.moveFeet(1.3, this, leftMotor, rightMotor)

        sleep(100)

        markerServo.position = 0.0
        sleep(2000)

        /*
        val turnTime = when {
            angle > (-3.0/2) * Math.PI -> 1850
            angle > (-2.0/3) * Math.PI -> 1850
            else -> 500
        }

        telemetry.addData("Angle", angle)
        telemetry.addData("TurnTime", turnTime)
        telemetry.update()

        Util.turnTime(turnTime.toLong(), 0.3, this, leftMotor, rightMotor)

        val feetToMove = when (turnTime) {
            1700 -> -3.5
            1200 -> -3.0
            else -> -2.5
        }

        Util.moveFeet(feetToMove, this, leftMotor, rightMotor)

        Util.turnTime(1200, 0.3, this, leftMotor, rightMotor)

        Util.moveFeet(-4.0, this, leftMotor, rightMotor)

        markerServo.position = 0.0

        sleep(500)

        Util.turnTime(200, -0.3, this, leftMotor, rightMotor)

        Util.moveFeet(8.0, this, leftMotor, rightMotor)
        /*val goToDepot = FollowConstantCurvaturePath(
                LinearPath(Waypoint(0.0, 0.0), Waypoint(0.0, 3.9)))

        updateCommandState(goToDepot)*/*/
    }

    private fun updateCommandState(command: Command) {
        while (!command.isCompleted() && opModeIsActive()) {
            SchedulerImpl.periodic()

            telemetry.update()
        }

        if (command.isCompleted()) {
            SchedulerImpl.periodic()
        }
        else {
            command.end()
        }
    }

    fun turnToAngle(target: Double) {
        while(opModeIsActive()) {
            val angle = imu.getRadians()

            val error = angle - target

            val output = error * 8

            leftMotor.power = output
            rightMotor.power = -output

            if (Math.abs(error) < 0.3) break


            telemetry.addData("Radians", angle)

            telemetry.update()
        }
    }
}