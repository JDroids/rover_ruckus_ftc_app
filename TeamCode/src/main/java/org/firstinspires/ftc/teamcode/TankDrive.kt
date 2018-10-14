package org.firstinspires.ftc.teamcode

import com.jdroids.robotlib.command.Scheduler
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name="TankDrive")
class TankDrive : OpMode() {
    override fun init() {
        Robot.teleopInit()
    }

    override fun loop() {
        Robot.drive.leftPower = -gamepad1.left_stick_y.toDouble()
        Robot.drive.leftPower = -gamepad1.right_stick_y.toDouble()
        Scheduler.run()
    }
}