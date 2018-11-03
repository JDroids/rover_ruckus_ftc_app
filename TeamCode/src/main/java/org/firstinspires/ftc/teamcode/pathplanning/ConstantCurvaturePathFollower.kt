package org.firstinspires.ftc.teamcode.pathplanning

class ConstantCurvaturePathFollower
    (val path: ConstantCurvaturePath, val constraints: MotionProfilingConstraints, val statistics: DriveTrainStatistics) : PathFollower {

    private fun cartesianDistance(point1: Waypoint, point2: Waypoint) =
            Math.sqrt(
                    Math.pow(point2.x - point1.x, 2.0) +
                    Math.pow(point2.y - point1.y, 2.0)
            )

    private val maxVelocityAchieved =
            if (cartesianDistance(path.generate(0.0), path.generate(0.5)) <
                    constraints.maximumVelocity) cartesianDistance(path.generate(0.0),
                    path.generate(0.5)) else constraints.maximumVelocity

    private val timeToAccelerate = maxVelocityAchieved/constraints.maximumAcceleration

    override val timeToFollow = timeToAccelerate + (path.length/maxVelocityAchieved)

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
