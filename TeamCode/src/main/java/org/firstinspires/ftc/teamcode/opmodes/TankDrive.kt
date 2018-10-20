package org.firstinspires.ftc.teamcode.opmodes

import com.jdroids.robotlib.command.Scheduler
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robot.Robot

@Disabled
@TeleOp(name="TankDrive")
class TankDrive : OpMode() {
    override fun init() {
        Robot.initHardware()
        Scheduler.enable()
    }

    override fun loop() {
        Robot.drive.leftPower = -gamepad1.left_stick_y.toDouble()
        Robot.drive.rightPower = -gamepad1.right_stick_y.toDouble()
        Scheduler.run()
    }
}