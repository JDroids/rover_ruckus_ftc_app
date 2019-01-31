package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.jdroids.robotlib.controller.PIDControllerImpl
//import com.disnodeteam.dogecv.CameraViewDisplay
//import com.disnodeteam.dogecv.DogeCV
//import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.*
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector
import org.firstinspires.ftc.teamcode.pathplanning.*
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.SamplingHelper
import org.firstinspires.ftc.teamcode.robot.commands.TurnToAngle
import kotlin.math.roundToInt

object Util {
    fun Number.toRadians() = this.toDouble() * (Math.PI / 180)
    fun Number.toDegrees() = this.toDouble() * (180 / Math.PI)

    fun BNO055IMU.getRadians() = ((this.angularOrientation.toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES).firstAngle + 180)).toRadians()

    val hitSample = LinearPath(Waypoint(0.0, 0.0), Waypoint(0.0, 1.0))
    val depositMarker = LinearPath(Waypoint(0.0, 0.0), Waypoint(0.0, 2.5))

    val goToCrater = LinearPath(Waypoint(0.0, 0.0), Waypoint(0.0, 7.5))

    private val statistics = DriveTrainStatistics(1.0 / 3.0, 1.145833)
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

    fun land(opMode: LinearOpMode, hangMotor1: DcMotor, hangMotor2: DcMotor,
             magnetSensor: DigitalChannel) {
        hangMotor1.power = 0.9
        hangMotor2.power = -0.9

        while (magnetSensor.state && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Magnet State", magnetSensor.state)
            opMode.telemetry.update()
        }

        hangMotor1.power = 0.0
        hangMotor2.power = 0.0

    }

    fun claim() {

    }

    enum class HookState {
        LATCHED,
        OPENED
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

    fun followPath(path: ConstantCurvaturePath, opMode: LinearOpMode, leftMotor: DcMotor, rightMotor: DcMotor, reversed: Boolean = false) {
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

    fun encoderTicksToFeet(ticks: Int) = ((ticks / 560.0) * 4 * Math.PI) / 12
    fun feetToEncoderTicks(feet: Double) = ((12.0 * feet) / (4 * Math.PI) * 560.0).roundToInt()

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
        @JvmField
        var p = 0.05
        @JvmField
        var i = 0.0
        @JvmField
        var d = 0.0
    }

    fun moveFeet(feet: Double, powerMax: Double, opMode: LinearOpMode, leftMotor1: DcMotor, leftMotor2: DcMotorEx, rightMotor1: DcMotorEx, rightMotor2: DcMotorEx) {
        val ticks = feetToEncoderTicks(-feet)

        val motors = listOf(leftMotor1, leftMotor2, rightMotor1, rightMotor2)

        motors.forEach {
            it.apply {
                mode = DcMotor.RunMode.RUN_TO_POSITION
                targetPosition = currentPosition + ticks
                power = powerMax
            }
        }

        while (!motors.all {!it.isBusy} && opMode.opModeIsActive()) {
        }

        motors.forEach {
            it.apply {
                power = 0.0
                mode = DcMotor.RunMode.RUN_USING_ENCODER
            }
        }
    }

    fun turnTime(ms: Long, power: Double, opMode: LinearOpMode,
                 leftMotor: DcMotor, rightMotor: DcMotor) {
        leftMotor.power = power
        rightMotor.power = -power

        opMode.sleep(ms)

        leftMotor.power = 0.0
        rightMotor.power = 0.0
    }

    fun normalizeAngle(angle: Double, angleUnit: AngleUnit): Double {
        val TAU = Math.PI * 2

        var angle = if(angleUnit == AngleUnit.RADIANS) angle else Math.toRadians(angle)

        angle = (angle % TAU)
        angle = (angle + TAU) % TAU

        if (angle > Math.PI)
            angle -= TAU
        return if(angleUnit == AngleUnit.RADIANS) angle else Math.toDegrees(angle)
    }
    fun turnToAngle(angleUnit: AngleUnit, angle: Double, opMode: LinearOpMode,
                    leftMotor1: DcMotorEx, leftMotor2: DcMotorEx,
                    rightMotor1: DcMotorEx, rightMotor2: DcMotorEx, imu: BNO055IMU) {

        var p = 10.0

        val angleRadians = normalizeAngle(angleUnit.toRadians(angle), AngleUnit.RADIANS)

        var output = 0.0

        val controller = PIDControllerImpl(
                { normalizeAngle(imu.getRadians(), AngleUnit.RADIANS) },
                { o: Double -> output = o},
                angleRadians,
                TurnToAngle.TurningCoefficients.p,
                TurnToAngle.TurningCoefficients.i,
                TurnToAngle.TurningCoefficients.d
        )

        do {
            controller.result()

            opMode.telemetry.addData("Output", output)

            setMotorVelocity(MotorVelocity(-output, output),
                    leftMotor1, leftMotor2, rightMotor1, rightMotor2)

            opMode.telemetry.update()
        }
        while (Math.abs(imu.getRadians() - angleRadians) >= (Math.PI/64)
                && output >= 0.1 && opMode.opModeIsActive())


        setMotorVelocity(MotorVelocity(0.0, 0.0),
                leftMotor1, leftMotor2, rightMotor1, rightMotor2)
    }

    fun getToDistanceWithDistanceSensor(inches: Double, linearOpMode: LinearOpMode,
                                        sensor: DistanceSensor, leftMotor: DcMotor,
                                        rightMotor: DcMotor) {
        while (linearOpMode.opModeIsActive()) {
            if (sensor.getDistance(DistanceUnit.INCH) - inches < 2) {
                leftMotor.power = -0.4
                rightMotor.power = -0.4
            } else if (sensor.getDistance(DistanceUnit.INCH) - inches > 2) {
                leftMotor.power = 0.4
                rightMotor.power = 0.4
            } else {
                break
            }
        }
    }

    fun setMotorVelocity(motorVelocity: MotorVelocity, leftMotor1: DcMotorEx, leftMotor2: DcMotorEx,
                         rightMotor1: DcMotorEx, rightMotor2: DcMotorEx) {
        val wheelCircumference = statistics.wheelRadius * Math.PI * 2.0

        val leftMotorVelocity = motorVelocity.leftVelocity / wheelCircumference / Math.PI * 2
        val rightMotorVelocity = motorVelocity.rightVelocity / wheelCircumference / Math.PI * 2


        leftMotor1.setVelocity(leftMotorVelocity, AngleUnit.RADIANS)
        leftMotor2.setVelocity(leftMotorVelocity, AngleUnit.RADIANS)

        rightMotor1.setVelocity(rightMotorVelocity, AngleUnit.RADIANS)
        rightMotor2.setVelocity(rightMotorVelocity, AngleUnit.RADIANS)
    }


    fun turnToGold(opMode: LinearOpMode, helper: SamplingHelper, leftMotor1: DcMotorEx,
                   leftMotor2: DcMotorEx, rightMotor1: DcMotorEx, rightMotor2: DcMotorEx) {
        val P_COEFFICIENT = 25
        val SCANNING_POWER = 6.0

        val timer = ElapsedTime()

        while (Math.abs(helper.goldAngle) > 0.03 && opMode.opModeIsActive()) {
            helper.update()

            var result = if (helper.goldAngle != -1.0) helper.goldAngle * P_COEFFICIENT else 0.0

            if (helper.goldAngle == -1.0) {

                result = when {
                    timer.seconds() >= 6 -> {timer.reset(); 0.0}
                    timer.seconds() >= 5 -> -SCANNING_POWER
                    timer.seconds() >= 3 -> SCANNING_POWER
                    timer.seconds() >= 2 -> -SCANNING_POWER
                    else -> result
                }
            }

            setMotorVelocity(MotorVelocity(result, -result), leftMotor1, leftMotor2,
                    rightMotor1, rightMotor2)

            opMode.telemetry.addData("Gold Angle", helper.goldAngle)
            opMode.telemetry.addData("Result", result)
            opMode.telemetry.update()
        }

        leftMotor1.power = 0.0
        leftMotor2.power = 0.0
        rightMotor1.power = 0.0
        rightMotor2.power = 0.0

        helper.kill()
    }
}