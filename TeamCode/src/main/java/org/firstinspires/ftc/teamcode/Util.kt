package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
//import com.disnodeteam.dogecv.CameraViewDisplay
//import com.disnodeteam.dogecv.DogeCV
//import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.pathplanning.*
import kotlin.math.roundToInt

object Util {
    fun Number.toRadians() = this.toDouble() * (Math.PI/180)
    fun Number.toDegrees() = this.toDouble() * (180/Math.PI)

    fun BNO055IMU.getRadians() = ((this.angularOrientation.toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES).firstAngle+180)*-1).toRadians()

    val hitSample = LinearPath(Waypoint(0.0, 0.0), Waypoint(0.0, 1.0))
    val depositMarker = LinearPath(Waypoint(0.0, 0.0), Waypoint(0.0, 2.5))

    val goToCrater = LinearPath(Waypoint(0.0, 0.0), Waypoint(0.0, 7.5))

    private val statistics = DriveTrainStatistics(1.0/3.0, 1.145833)
    private val constraints = MotionProfilingConstraints(2.3, 0.75)

    val leftSampleRelativeAngle = (-30.0).toRadians()
    val centerSampleRelativeAngle = 0.0.toRadians()
    val rightSampleRelativeAngle = 30.0.toRadians()

    fun initializeIMU(imu: BNO055IMU) {
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.calibrationDataFile = "BNO055IMUCalibration.json" // see the calibration sample opmode
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"

        imu.initialize(parameters)
    }

    fun land(opMode: LinearOpMode, hangMotor1: DcMotor, hangMotor2: DcMotor, hookServo: Servo,
             tofSensor: Rev2mDistanceSensor) {
        hangMotor1.power = 0.5
        hangMotor2.power = 0.5

        while (tofSensor.getDistance(DistanceUnit.INCH) > 1.8 && opMode.opModeIsActive()) {
            opMode.telemetry.addData("TOFSensor", tofSensor.getDistance(DistanceUnit.INCH))
            opMode.telemetry.update()
        }

        opMode.sleep(25)

        hangMotor1.power = 0.0
        hangMotor2.power = 0.0

        setHookState(HookState.OPENED, hookServo)

        opMode.sleep(500)
    }

    fun claim() {

    }

    enum class HookState {
        LATCHED,
        OPENED
    }

    fun setHookState(state: HookState, servo: Servo) {
        servo.position = when (state) {
            HookState.LATCHED -> 0.7
            HookState.OPENED -> 0.0
        }
    }

    enum class MarkerState {
        HOLDING,
        OPENED
    }

    fun setMarkerState(state: MarkerState, servo: Servo) {
        servo.position = when (state) {
            MarkerState.HOLDING -> 0.7
            MarkerState.OPENED -> 0.0
        }
    }

    /*fun doVision(hardwareMap: HardwareMap): SamplingVision.GoldPosition {
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
    }*/

   /* fun hitSampleAndDepositMarker(samplePosition: SamplingVision.GoldPosition, opMode: LinearOpMode,
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
        /*turnToRelativeAngle(when (samplePosition) {
            SamplingVision.GoldPosition.LEFT -> leftSampleRelativeAngle
            SamplingVision.GoldPosition.CENTER -> centerSampleRelativeAngle
            SamplingVision.GoldPosition.RIGHT -> rightSampleRelativeAngle
            else -> throw IllegalArgumentException("None position passed")
            }, opMode, leftMotor, rightMotor, imu)
        */
        followPath(hitSample, opMode, leftMotor, rightMotor)
    }*/

    fun followPath(path: ConstantCurvaturePath, opMode: LinearOpMode, leftMotor: DcMotor, rightMotor: DcMotor, reversed:Boolean=false) {
        val follower = ConstantCurvaturePathFollower(path, constraints, statistics)

        val timer = ElapsedTime()

        val direction = if (reversed) -1 else 1

        while (timer.seconds() < follower.timeToFollow && opMode.opModeIsActive()) {
            follower.generate(timer.seconds())

            val followerResult = follower.generate(timer.seconds())
            val target = followerResult.state.toMotorVelocity(statistics)

            leftMotor.power = (target.leftVelocity / constraints.maximumVelocity) * direction
            rightMotor.power = (target.leftVelocity / constraints.maximumVelocity) * direction
        }
    }

    fun encoderTicksToFeet(ticks: Int) = ((ticks/1120.0) * 4 * Math.PI) / 12
    fun feetToEncoderTicks(feet: Double) = ((1120.0*12.0*feet) / (4 * Math.PI)).roundToInt()

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
        @JvmField var p = 0.05
        @JvmField var i = 0.0
        @JvmField var d = 0.0
    }

    fun moveFeet(feet: Double, opMode: LinearOpMode, leftMotor: DcMotor, rightMotor: DcMotor) {
        val ticks = feetToEncoderTicks(feet)

        leftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        rightMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        leftMotor.targetPosition = leftMotor.currentPosition + ticks
        rightMotor.targetPosition = rightMotor.currentPosition + ticks

        leftMotor.power = 1.0
        rightMotor.power = 1.0

        while ((leftMotor.isBusy || rightMotor.isBusy) && opMode.opModeIsActive()) { }

        leftMotor.power = 0.0
        rightMotor.power = 0.0

        leftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun turnTime(ms: Long, power: Double, opMode: LinearOpMode,
                           leftMotor: DcMotor, rightMotor: DcMotor) {
        leftMotor.power = power
        rightMotor.power = -power

        opMode.sleep(ms)

        leftMotor.power = 0.0
        rightMotor.power = 0.0
    }

    fun turnToAngle(angle: Double, opMode: LinearOpMode,
                    leftMotor: DcMotor, rightMotor: DcMotor, imu: BNO055IMU) {
        val pid = JankPID()

        pid.setCoeffecients(TurningPIDCoefficients.p, TurningPIDCoefficients.i,
                TurningPIDCoefficients.d)

        var current = imu.getRadians()

        while (Math.abs(current - angle) > 7.toRadians() && opMode.opModeIsActive()) {
            current = imu.getRadians()

            val output = pid.calculateOutput(angle, current)

            leftMotor.power = output
            rightMotor.power = -output

            opMode.telemetry.addData("Output", output)
            opMode.telemetry.addData("Error", current - angle)
            opMode.telemetry.addData("Left Motor Power", leftMotor.power)
            opMode.telemetry.addData("Right Motor Power", rightMotor.power)
            opMode.telemetry.update()
        }

        leftMotor.power = 0.0
        rightMotor.power = 0.0
    }

    fun turnToRelativeAngle(angle: Double, opMode: LinearOpMode,
                    leftMotor: DcMotor, rightMotor: DcMotor, imu: BNO055IMU) {
        turnToAngle(angle + imu.getRadians(), opMode, leftMotor, rightMotor, imu)
    }

    fun getToDistanceWithDistanceSensor(inches: Double, linearOpMode: LinearOpMode,
                                        sensor: DistanceSensor, leftMotor: DcMotor,
                                        rightMotor: DcMotor) {
        while (linearOpMode.opModeIsActive()) {
            if (sensor.getDistance(DistanceUnit.INCH) - inches < 2) {
                leftMotor.power = -0.4
                rightMotor.power = -0.4
            }
            else if(sensor.getDistance(DistanceUnit.INCH) - inches > 2) {
                leftMotor.power = 0.4
                rightMotor.power = 0.4
            }
            else {
                break
            }
        }
    }
}