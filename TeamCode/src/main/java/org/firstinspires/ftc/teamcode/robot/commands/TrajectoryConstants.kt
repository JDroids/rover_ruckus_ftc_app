package org.firstinspires.ftc.teamcode.robot.commands

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.jdroids.robotlib.command.Command
import com.acmerobotics.roadrunner.control.PIDCoefficients
import org.firstinspires.ftc.teamcode.robot.Robot

class FollowTrajectory(initialPose: Pose2d, targetPose: Pose2d) : Command {
    @Config
    object TrajectoryConstants {
        @JvmField var MAX_VELOCITY = 0.0
        @JvmField var MAX_ACCELERATION = 0.0

        @JvmField var MAX_ANGULAR_VELOCITY = 0.0
        @JvmField var MAX_ANGULAR_ACCELERATION = 0.0

        @JvmField var DISPLACEMENT_COEFFICIENTS = PIDCoefficients(0.0, 0.0, 0.0)

        @JvmField var CROSS_TRACK_COEFFICIENTS = PIDCoefficients(0.0, 0.0, 0.0)

        @JvmField var kStatic = 0.0
    }

    val driveWrapper = Robot.drive.DriveWrapper()

    private val trajectory = TrajectoryBuilder(
        initialPose,
        DriveConstraints(
            TrajectoryConstants.MAX_VELOCITY,
            TrajectoryConstants.MAX_ACCELERATION,
            TrajectoryConstants.MAX_ANGULAR_VELOCITY,
            TrajectoryConstants.MAX_ANGULAR_ACCELERATION
        )
    ).splineTo(targetPose).build()

    private val follower = TankPIDVAFollower(
        driveWrapper,
        TrajectoryConstants.DISPLACEMENT_COEFFICIENTS,
        TrajectoryConstants.CROSS_TRACK_COEFFICIENTS,
        1/TrajectoryConstants.MAX_VELOCITY,
        1/TrajectoryConstants.MAX_ACCELERATION,
        TrajectoryConstants.kStatic
    )

    override fun start() {
        follower.followTrajectory(trajectory)
    }

    override fun periodic() {
        driveWrapper.updatePoseEstimate()
        follower.update(driveWrapper.poseEstimate)
    }

    override fun end() {

    }

    override fun isCompleted() = follower.isFollowing()
}