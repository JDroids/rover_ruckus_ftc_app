package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import java.io.FileWriter
import android.R.attr.path
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.hardware.DcMotor
import java.io.File
import java.io.FileOutputStream

@Disabled
@TeleOp(name="TestNyoooooooooooooommmmmmmmmm")
class TestNyoooooooooooooommmmmmmmmm : LinearOpMode() {
    private val leftMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "left")}
    private val rightMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "right")}

    override fun runOpMode() {
        telemetry.addData("File Dir: ", hardwareMap.appContext.filesDir)
        telemetry.update()
        leftMotor.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        leftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        leftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        leftMotor.power = 1.0
        rightMotor.power = 1.0

        val elapsedTime = ElapsedTime()

        val stream = FileOutputStream(File("/sdcard/CSVFiles", "Speed.csv"))

        while (opModeIsActive()) {
            stream.write("${elapsedTime.milliseconds()},${leftMotor.currentPosition},${rightMotor.currentPosition}\n".toByteArray())
        }

        stream.close()

        leftMotor.power = 0.0
        rightMotor.power = 0.0
    }
}