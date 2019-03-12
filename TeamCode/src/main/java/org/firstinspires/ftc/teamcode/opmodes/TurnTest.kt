package org.firstinspires.ftc.teamcode.opmodes

import com.jdroids.robotlib.command.SchedulerImpl
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Util
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.commands.TurnToAngle
import org.firstinspires.ftc.teamcode.Util.toRadians

//@Disabled
@Autonomous(name="TurnTest")
class TurnTest : LinearOpMode() {
    private val imu by lazy {hardwareMap!!.get(BNO055IMU::class.java, "imu")}

    private val leftFrontMotor
            by lazy { hardwareMap.get(DcMotorEx::class.java, "lf") }
    private val leftBackMotor
            by lazy { hardwareMap.get(DcMotorEx::class.java, "lb") }

    private val rightFrontMotor
            by lazy { hardwareMap.get(DcMotorEx::class.java, "rf") }
    private val rightBackMotor
            by lazy { hardwareMap.get(DcMotorEx::class.java, "rb") }

    override fun runOpMode() {
        rightFrontMotor.direction = DcMotorSimple.Direction.REVERSE
        rightBackMotor.direction = DcMotorSimple.Direction.REVERSE

        Util.initializeIMU(imu)

        waitForStart()

        Util.turnToAngle(AngleUnit.DEGREES, 39.0, this, leftFrontMotor, leftBackMotor,
                rightFrontMotor, rightBackMotor, imu)

        Util.turnToAngle(AngleUnit.DEGREES, 90.0, this, leftFrontMotor, leftBackMotor,
                rightFrontMotor, rightBackMotor, imu)


        Util.turnToAngle(AngleUnit.DEGREES, 70.0, this, leftFrontMotor, leftBackMotor,
                rightFrontMotor, rightBackMotor, imu)

        Util.turnToAngle(AngleUnit.DEGREES, 0.0, this, leftFrontMotor, leftBackMotor,
                rightFrontMotor, rightBackMotor, imu)

        Util.turnToAngle(AngleUnit.DEGREES, 180.0, this, leftFrontMotor, leftBackMotor,
                rightFrontMotor, rightBackMotor, imu)
    }
}