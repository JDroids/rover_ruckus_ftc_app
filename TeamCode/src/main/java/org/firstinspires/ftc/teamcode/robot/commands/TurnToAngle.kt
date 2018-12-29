package org.firstinspires.ftc.teamcode.robot.commands

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.Util.getRadians
import org.firstinspires.ftc.teamcode.Util.toRadians
import com.jdroids.robotlib.command.Command
import com.jdroids.robotlib.controller.PIDControllerImpl
import com.qualcomm.robotcore.hardware.PIDCoefficients
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.pathplanning.MotorVelocity
import org.firstinspires.ftc.teamcode.robot.Robot

class TurnToAngle(angleUnit: AngleUnit, value: Double) : Command {
    private val angle = AngleUnit.RADIANS.fromUnit(angleUnit, value)

    @Config
    object TurningCoefficients {
        @JvmField var p = 10.0
        @JvmField var i = 0.0
        @JvmField var d = -0.5
    }

    private val controller = PIDControllerImpl(
        {Robot.drive.imu.getRadians()},
        {o: Double -> Robot.drive.motorVelocity = MotorVelocity(-o, o)},
        angle,
        TurningCoefficients.p,
        TurningCoefficients.i,
        TurningCoefficients.d
    )

    override fun start() {
        //controller.setContinuous(0.0, Math.PI * 2)
    }

    override fun periodic() {
        controller.p = TurningCoefficients.p
        controller.i = TurningCoefficients.i
        controller.d = TurningCoefficients.d

        val result = controller.result()

        FtcDashboard.getInstance().telemetry.addData("setpoint", controller.setpoint)
        FtcDashboard.getInstance().telemetry.addData("current", Robot.drive.imu.getRadians() )
        FtcDashboard.getInstance().telemetry.addData("result", result)
        FtcDashboard.getInstance().telemetry.addData("lastError", controller.lastError)
        FtcDashboard.getInstance().telemetry.update()
    }

    override fun end() {
        Robot.drive.motorVelocity = MotorVelocity(0.0, 0.0)
    }

    override fun isCompleted() = Math.abs(angle - Robot.drive.imu.getRadians()) < (Math.PI/64)
}