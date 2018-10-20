package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.disnodeteam.dogecv.CameraViewDisplay
import com.disnodeteam.dogecv.DogeCV
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector
import com.jdroids.robotlib.pid.PIDInterface
import com.jdroids.robotlib.pid.SimplePIDController
import com.jdroids.robotlib.util.Boundary
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.pathplanning.*

object Util {
    fun Number.toRadians() = this.toDouble() * (Math.PI/180)
    fun Number.toDegrees() = this.toDouble() * (180/Math.PI)

    fun BNO055IMU.getRadians() = ((this.angularOrientation.toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES).firstAngle+180)*-1).toRadians()

    val hitSample = LinearPath(Waypoint(0.0, 0.0), Waypoint(0.0, 2.5))
    val depositMarker = LinearPath(Waypoint(0.0, 0.0), Waypoint(0.0, 2.5))

    val goToCrater = LinearPath(Waypoint(0.0, 0.0), Waypoint(0.0, 7.5))

    private val statistics = DriveTrainStatistics(1.0/3.0, 1.145833)
    private val constraints = MotionProfilingConstraints(2.3, 0.75)

    val leftSampleRelativeAngle = (-30.0).toRadians()
    val centerSampleRelativeAngle = 0.0.toRadians()
    val rightSampleRelativeAngle = 30.0.toRadians()

    fun initializeIMU(imu: BNO055IMU) {
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.calibrationDataFile = "BNO055IMUCalibration.json" // see the calibration sample opmode
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"

        imu.initialize(parameters)
    }

    fun doVision(hardwareMap: HardwareMap): SamplingVision.GoldPosition {
        val detector = SamplingOrderDetector()

        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1,
                false)
        detector.useDefaults()
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA
        detector.maxAreaScorer.weight = 0.001
        detector.ratioScorer.weight = 15.0
        detector.ratioScorer.perfectRatio = 1.0
        detector.enable()

        var currentOrder = detector.currentOrder
        while (currentOrder == SamplingOrderDetector.GoldLocation.UNKNOWN) {
            currentOrder = detector.currentOrder
        }

        return when (currentOrder) {
            SamplingOrderDetector.GoldLocation.UNKNOWN -> SamplingVision.GoldPosition.NONE
            SamplingOrderDetector.GoldLocation.LEFT -> SamplingVision.GoldPosition.LEFT
            SamplingOrderDetector.GoldLocation.CENTER -> SamplingVision.GoldPosition.CENTER
            SamplingOrderDetector.GoldLocation.RIGHT -> SamplingVision.GoldPosition.RIGHT
            null -> throw NullPointerException()
        }
    }

    fun hitSampleAndDepositMarker(samplePosition: SamplingVision.GoldPosition, opMode: LinearOpMode,
                                  leftMotor: DcMotor, rightMotor: DcMotor, imu: BNO055IMU) {
        hitSample(samplePosition, opMode, leftMotor, rightMotor, imu)

        when (samplePosition) {
            SamplingVision.GoldPosition.LEFT -> turnToRelativeAngle(30.toRadians(), opMode,
                    leftMotor, rightMotor, imu)
            SamplingVision.GoldPosition.RIGHT -> turnToRelativeAngle((-30).toRadians(), opMode,
                    leftMotor, rightMotor, imu)
            else -> {}
        }

        followPath(depositMarker, opMode, leftMotor, rightMotor)
    }

    fun hitSample(samplePosition: SamplingVision.GoldPosition, opMode: LinearOpMode,
                  leftMotor: DcMotor, rightMotor: DcMotor, imu: BNO055IMU) {
        turnToRelativeAngle(when (samplePosition) {
            SamplingVision.GoldPosition.LEFT -> leftSampleRelativeAngle
            SamplingVision.GoldPosition.CENTER -> centerSampleRelativeAngle
            SamplingVision.GoldPosition.RIGHT -> rightSampleRelativeAngle
            else -> throw IllegalArgumentException("None position passed")
            }, opMode, leftMotor, rightMotor, imu)

        followPath(hitSample, opMode, leftMotor, rightMotor)
    }

    fun followPath(path: ConstantCurvaturePath, opMode: LinearOpMode, leftMotor: DcMotor, rightMotor: DcMotor) {
        val follower = ConstantCurvaturePathFollower(path, constraints, statistics)

        val timer = ElapsedTime()

        while (timer.seconds() < follower.timeToFollow && opMode.opModeIsActive()) {
            follower.generate(timer.seconds())

            val followerResult = follower.generate(timer.seconds())
            val target = followerResult.state.toMotorVelocity(statistics)

            leftMotor.power = (target.leftVelocity / constraints.maximumVelocity)
            rightMotor.power = (target.leftVelocity / constraints.maximumVelocity)
        }
    }

    fun encoderTicksToFeet(ticks: Int) = ((ticks/1120.0) * 4 * Math.PI) / 12

    data class Position(val waypoint: Waypoint, val angle: Double)

    fun odometry(currentPosition: Position, deltaLeftTicks: Int, deltaRightTicks: Int,
                 statistics: DriveTrainStatistics): Position {
        val deltaLeft = encoderTicksToFeet(deltaLeftTicks)
        val deltaRight = encoderTicksToFeet(deltaRightTicks)

        val vs = (deltaLeft + deltaRight) / 2
        val omega = (deltaLeft + deltaRight) / statistics.wheelDistance

        val angle = currentPosition.angle + omega
        val x = Math.cos(vs) * angle
        val y = Math.sin(vs) * angle

        return Position(Waypoint(x, y), angle)
    }

    @Config
    object TurningPIDCoefficients {
        @JvmField var coefficients = PIDCoefficients(0.0, 0.0, 0.0)
    }

    fun turnToAngle(angle: Double, opMode: LinearOpMode,
                    leftMotor: DcMotor, rightMotor: DcMotor, imu: BNO055IMU) {
        val pid = SimplePIDController({imu.getRadians()},
                {output -> leftMotor.power = output; rightMotor.power = -output},
                TurningPIDCoefficients.coefficients)

        pid.setpoint = imu.getRadians()
        pid.continuous = true
        pid.inputBoundary = Boundary(0.0, Math.PI*2)
        pid.tolerance = PIDInterface.Tolerance.percentageTolerance(5.0)

        while (opMode.opModeIsActive()) {
            pid.calculate()
        }
    }

    fun turnToRelativeAngle(angle: Double, opMode: LinearOpMode,
                    leftMotor: DcMotor, rightMotor: DcMotor, imu: BNO055IMU) {
        turnToAngle(angle + imu.getRadians(), opMode, leftMotor, rightMotor, imu)
    }
}