package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Util
import org.firstinspires.ftc.teamcode.Util.getRadians

@Disabled
@TeleOp(name="ImuTest")
class ImuTest : LinearOpMode() {
    private val imu by lazy {hardwareMap!!.get(BNO055IMU::class.java, "imu")}

    override fun runOpMode() {
        Util.initializeIMU(imu)

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("Angle", imu.getRadians())
            telemetry.update()
        }
    }
}