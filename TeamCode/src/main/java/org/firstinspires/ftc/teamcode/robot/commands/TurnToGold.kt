package org.firstinspires.ftc.teamcode.robot.commands

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.jdroids.robotlib.command.Command
import com.jdroids.robotlib.command.SchedulerImpl
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector
import org.firstinspires.ftc.teamcode.JankPID
import org.firstinspires.ftc.teamcode.pathplanning.MotorVelocity
import org.firstinspires.ftc.teamcode.robot.Robot

class TurnToGold : Command {
    @Config
    object TurnCoefficients {
        @JvmField var TURNING_PID = PIDCoefficients(-25.0, 0.0, -1.0)
    }

    private val TFOD_MODEL_ASSET = "RoverRuckus.tflite"
    private val GOLD_MINERAL_LABEL = "Gold Mineral"

    private val tfod = getTfod(getVuforia())

    private val pid = JankPID()

    private val timer = ElapsedTime()

    /*private val pid = PIDControllerImpl(
        {goldAngle},
        {power -> Robot.drive.motorVelocity = MotorVelocity(power, -power)},
        0.0,
        TurnCoefficients.TURNING_PID.p,
        TurnCoefficients.TURNING_PID.i,
        TurnCoefficients.TURNING_PID.d
    )*/

    override fun start() {
        tfod.activate()
        timer.reset()
    }

    private var goldAngle = -1.0

    private var isFirstTimeDetected = true

    override fun periodic() {
        val recognitions = tfod.updatedRecognitions

        if (recognitions != null) {
            for (recognition in recognitions) {
                if (recognition.label == GOLD_MINERAL_LABEL) {
                    goldAngle = recognition.estimateAngleToObject(AngleUnit.RADIANS)

                    if (isFirstTimeDetected) {
                        isFirstTimeDetected = false
                    }
                }
            }
        }

        val coefficient = TurnCoefficients.TURNING_PID.p

        var result = if (goldAngle != -1.0) goldAngle * coefficient else 0.0

        if (goldAngle == -1.0) {
            if (timer.seconds() >= 8) {
                timer.reset()
            }
            else if (timer.seconds() >= 4) {
                result = 4.0
            }
            else if (timer.seconds() >= 2) {
                result = -4.0
            }
        }

        Robot.drive.motorVelocity = MotorVelocity(-result, result)

        FtcDashboard.getInstance().telemetry.addData("Gold Location", goldAngle)
        FtcDashboard.getInstance().telemetry.addData("PIDResult", result)
        FtcDashboard.getInstance().telemetry.update()
    }

    override fun end() {
        if (tfod != null) tfod.shutdown()

        Robot.drive.motorVelocity = MotorVelocity(0.0, 0.0)

        Log.d("TurnToGold", "End called")
    }

    override fun isCompleted() = Math.abs(goldAngle) < 0.1

    private fun getTfod(vuforia: VuforiaLocalizer) : TFObjectDetector {
        val tfodMonitorViewId = Robot.opMode.hardwareMap.appContext.resources.getIdentifier(
                "tfodMonitorViewId", "id",
                Robot.opMode.hardwareMap.appContext.packageName
        )

        val params = TFObjectDetector.Parameters(tfodMonitorViewId)

        val tfod = ClassFactory.getInstance().createTFObjectDetector(params, vuforia)

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, GOLD_MINERAL_LABEL)

        return tfod
    }

    private fun getVuforia() : VuforiaLocalizer {
        val params = VuforiaLocalizer.Parameters()

        params.vuforiaLicenseKey = " Af8z1N//////AAABmd+VTcKIy0DvswaS6ptJxhU6esp8q/iwhtFaV1BcqNpTKe5OuZmOsRDT7ThrIx4/49OsRIPgC18aN8v93oqt/F0IGHy32sgT5U3BV7xchvQ5uGUvACuy4+9wXouHBalSXYWX/bLd0hhYVx3oe+D/WqrhqmZTvLbjAdxRdecRc0wNDwUSN1Iz0dQR19h8TDdenzHR7vNBVAR44/X4c8fFuEnJ06lKxJqzunFAgsRmBt5uzG/HLg1vxRJDfX04pEDILoJKfG9hqI1Hx+MjBcdJj4WMLg43D9iokXSuc7I9SJiu7L6TwWutKeK9ANACkCdAN6UaYpNXFRf9pjvhCLeTa2mlWkuN7gIxeswkuL+x4qtQ"

        params.cameraName = Robot.opMode.hardwareMap.get(WebcamName::class.java, "Webcam 1")

        return ClassFactory.getInstance().createVuforia(params)
    }
}