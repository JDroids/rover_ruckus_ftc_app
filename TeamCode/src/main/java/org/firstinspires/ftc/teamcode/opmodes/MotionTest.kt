package org.firstinspires.ftc.teamcode.opmodes
/*
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.jdroids.robotlib.command.SchedulerImpl
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Drive
import org.firstinspires.ftc.teamcode.pathplanning.*

@Disabled
@Autonomous(name="MotionTest")
class MotionTest : LinearOpMode() {
    private val leftMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "left")}
    private val rightMotor by lazy {hardwareMap!!.get(DcMotorEx::class.java, "right")}

    private val statistics = DriveTrainStatistics(1.0/3.0, 1.145833)
    private val constraints =
            MotionProfilingConstraints(Drive.DrivetrainRoadrunnerCoefficients.maxVelocity,
                    Drive.DrivetrainRoadrunnerCoefficients.maxAcceleration)

    private val follower = ConstantCurvaturePathFollower(LinearPath(
            Waypoint(0.0, 0.0),
            Waypoint(0.0, 3.0)), constraints, statistics)

    override fun runOpMode() {
        leftMotor.direction = DcMotorSimple.Direction.REVERSE

        leftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        waitForStart()

        val timer = ElapsedTime()
        val packet = TelemetryPacket()

        while (timer.seconds() <= follower.timeToFollow && opModeIsActive()) {
            val followerResult = follower.generate(timer.seconds())
            val target = followerResult.state.toMotorVelocity(statistics)

            leftMotor.power = (target.leftVelocity / constraints.maximumVelocity)
            rightMotor.power = (target.leftVelocity / constraints.maximumVelocity)

            packet.put("Timer Seconds", timer.seconds())
            packet.put("Seconds Left", follower.timeToFollow - timer.seconds())
            packet.put("Target Position X", followerResult.waypoint.x)
            packet.put("Target Position Y", followerResult.waypoint.y)

            FtcDashboard.getInstance().sendTelemetryPacket(packet)

            SchedulerImpl.periodic()
        }

        SchedulerImpl.kill()
    }
}*/