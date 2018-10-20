package org.firstinspires.ftc.teamcode.pathplanning

data class MotionProfilingConstraints(val maximumVelocity: Double, val maximumAcceleration: Double)

data class PathFollowerResult(val state: DriveTrainState, val waypoint: Waypoint)

interface PathFollower {
    val timeToFollow: Double
    // t should range from 0 to the amount of time for the path in seconds
    fun generate(t: Double): PathFollowerResult
}