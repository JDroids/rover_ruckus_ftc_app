package org.firstinspires.ftc.teamcode.robot.commands

import com.acmerobotics.dashboard.config.Config
import com.disnodeteam.dogecv.CameraViewDisplay
import com.disnodeteam.dogecv.DogeCV
import com.disnodeteam.dogecv.Dogeforia
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector
import com.jdroids.robotlib.command.Command
import com.jdroids.robotlib.command.SchedulerImpl
import com.jdroids.robotlib.controller.PIDControllerImpl
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.teamcode.pathplanning.MotorVelocity
import org.firstinspires.ftc.teamcode.robot.Robot

class TurnToGold(private val opMode: OpMode) : Command {
    @Config
    object VisionConstants {
        @JvmField var p = 0.1
        @JvmField var perfectArea = 10000.0
        @JvmField var perfectAreaWeight = 0.001
    }

    private val vision = GoldAlignDetector()

    override fun start() {
        SchedulerImpl.requires(this, Robot.drive)

        val webcamName = opMode.hardwareMap.get(WebcamName::class.java, "Webcam 1")

        val parameters = VuforiaLocalizer.Parameters()

        parameters.vuforiaLicenseKey = " Af8z1N//////AAABmd+VTcKIy0DvswaS6ptJxhU6esp8q/iwhtFaV1BcqNpTKe5OuZmOsRDT7ThrIx4/49OsRIPgC18aN8v93oqt/F0IGHy32sgT5U3BV7xchvQ5uGUvACuy4+9wXouHBalSXYWX/bLd0hhYVx3oe+D/WqrhqmZTvLbjAdxRdecRc0wNDwUSN1Iz0dQR19h8TDdenzHR7vNBVAR44/X4c8fFuEnJ06lKxJqzunFAgsRmBt5uzG/HLg1vxRJDfX04pEDILoJKfG9hqI1Hx+MjBcdJj4WMLg43D9iokXSuc7I9SJiu7L6TwWutKeK9ANACkCdAN6UaYpNXFRf9pjvhCLeTa2mlWkuN7gIxeswkuL+x4qtQ "
        parameters.fillCameraMonitorViewParent = true
        parameters.cameraName = webcamName

        val vuforia = Dogeforia(parameters)
        vuforia.enableConvertFrameToBitmap()

        vision.init(opMode.hardwareMap.appContext, CameraViewDisplay.getInstance(),
                0, true)

        vision.useDefaults()

        vision.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA

        vision.perfectAreaScorer.perfectArea = VisionConstants.perfectArea
        vision.perfectAreaScorer.weight = VisionConstants.perfectAreaWeight

        vision.enable()
    }

    var timesPeriodicRun = 0
        private set

    override fun periodic() {
        val output = (vision.yPosition - (480/2)) * VisionConstants.p
        Robot.drive.motorVelocity = MotorVelocity(output, -output)
        ++timesPeriodicRun
        opMode.telemetry.addData("AlignPosOffset", vision.yPosition)
        opMode.telemetry.addData("PIDOutput", output)
        opMode.telemetry.addData("Times Periodic Ran", timesPeriodicRun)
    }

    override fun end() {
        Robot.drive.motorVelocity = MotorVelocity(0.0, 0.0)
        vision.disable()
    }

    override fun interrupt() {
        end()
    }

    override fun isCompleted() = Math.abs(vision.yPosition - 480/2) < 5

    override fun isInterruptible() = false
}