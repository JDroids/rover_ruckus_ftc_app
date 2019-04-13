package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.drive.TankDrive
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints
import org.firstinspires.ftc.teamcode.DriveConstants

class Drive(private val opMode: OpMode) : TankDrive(DriveConstants.TRACK_WIDTH) {
    private val leftFrontMotor by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "lf")}
    private val leftBackMotor by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "lb")}
    private val rightFrontMotor by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "rf")}
    private val rightBackMotor by lazy {opMode.hardwareMap.get(DcMotorEx::class.java, "rb")}

    private val follower = TankPIDVAFollower(
            this,
            DrivetrainRoadrunnerCoefficients.displacementCoeffs,
            DrivetrainRoadrunnerCoefficients.crossTrackCoeffs,
            DrivetrainRoadrunnerCoefficients.kV,
            DrivetrainRoadrunnerCoefficients.kA,
            DrivetrainRoadrunnerCoefficients.kStatic
    )

    fun followTrajectory(trajectory: Trajectory) {
        follower.followTrajectory(trajectory)
    }

    private val constraints = TankConstraints(
            DriveConstants.BASE_CONSTRAINTS, DriveConstants.TRACK_WIDTH)

    fun getTrajectoryBuilder() = TrajectoryBuilder(poseEstimate, constraints)

    //@Config
    object DrivetrainRoadrunnerCoefficients {
        @JvmField var displacementCoeffs = PIDCoefficients(0.0, 0.0, 0.0)
        @JvmField var crossTrackCoeffs = PIDCoefficients(0.0, 0.0, 0.0)
        @JvmField var kV = 0.0
        @JvmField var kA = 0.0
        @JvmField var kStatic = 0.0
        //@JvmField var maxVelocity = 36
        //@JvmField var maxAcceleration = 0.75
    }

    init {
        localizer = TankLocalizer(this, false)

        rightFrontMotor.direction = DcMotorSimple.Direction.REVERSE
        rightBackMotor.direction = DcMotorSimple.Direction.REVERSE

        leftFrontMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        leftBackMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightFrontMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightBackMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun isFollowing() = follower.isFollowing()

    fun update() {
        updatePoseEstimate()
        follower.update(localizer.poseEstimate)
    }

    override fun setMotorPowers(left: Double, right: Double) {
        leftFrontMotor.power = left
        leftBackMotor.power = left
        rightFrontMotor.power = right
        rightBackMotor.power = right

        opMode.telemetry.addData("Left", left)
        opMode.telemetry.addData("Right", right)
        opMode.telemetry.update()
    }

    override fun getExternalHeading(): Double {
        return 0.0
    }

    override fun getWheelPositions(): List<Double> =
            listOf(
                    DriveConstants.encoderTicksToInches(leftFrontMotor.currentPosition),
                    DriveConstants.encoderTicksToInches(rightFrontMotor.currentPosition)
            )
}