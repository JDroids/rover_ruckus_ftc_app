package org.firstinspires.ftc.teamcode.pathplanning

import android.util.Log

class ConstantCurvaturePathFollower
    (val path: ConstantCurvaturePath, val constraints: MotionProfilingConstraints, val statistics: DriveTrainStatistics) : PathFollower {

    private val timeToAccelerate = (constraints.maximumVelocity/constraints.maximumAcceleration)

    override val timeToFollow = timeToAccelerate + (path.length/constraints.maximumVelocity)

    private val timeToCruise = timeToFollow - timeToAccelerate

    override fun generate(t: Double): PathFollowerResult {
        val rightVelocity = when {
            t < timeToAccelerate -> constraints.maximumAcceleration * t
            t <= timeToCruise -> constraints.maximumVelocity
            t <= timeToFollow -> constraints.maximumVelocity -
                    (constraints.maximumAcceleration * (t - timeToCruise))
            else -> 0.0
        }

        val rightToLeftRatio = 1-path.curvature

        val rightVelocityScaled = if (rightToLeftRatio > -1) rightVelocity else Math.abs(rightVelocity/rightToLeftRatio) 

        return PathFollowerResult(
            MotorVelocity(rightVelocityScaled, rightVelocityScaled * rightToLeftRatio).toDriveTrainState(statistics),
            path.generate(t/timeToFollow)
        )
    }
}
