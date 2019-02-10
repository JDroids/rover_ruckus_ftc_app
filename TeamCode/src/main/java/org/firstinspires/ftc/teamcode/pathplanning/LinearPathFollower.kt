package org.firstinspires.ftc.teamcode.pathplanning

class LinearPathFollower
    (val target: Double, val constraints: MotionProfilingConstraints) : PathFollower {

    private val maxVelocityAchieved =
            if (target/2 * constraints.maximumVelocity < constraints.maximumVelocity)
                target/2 * constraints.maximumVelocity
            else constraints.maximumVelocity

    private val timeToAccelerate = maxVelocityAchieved/constraints.maximumAcceleration

    override val timeToFollow = timeToAccelerate + (target/maxVelocityAchieved)

    private val timeToCruise = timeToFollow - timeToAccelerate

    override fun generate(t: Double): PathFollowerResult {
        val velocity = when {
            t < timeToAccelerate -> constraints.maximumAcceleration * t
            t <= timeToCruise -> constraints.maximumVelocity
            t <= timeToFollow -> constraints.maximumVelocity -
                    (constraints.maximumAcceleration * (t - timeToCruise))
            else -> 0.0
        }

        return PathFollowerResult(
            DriveTrainState(velocity, 0.0),
            Waypoint(0.0, 0.0)
        )
    }
}
