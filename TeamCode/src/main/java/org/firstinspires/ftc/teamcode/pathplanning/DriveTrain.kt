package org.firstinspires.ftc.teamcode.pathplanning

data class DriveTrainStatistics(val wheelRadius: Double, val wheelDistance: Double)

data class DriveTrainState(val linearVelocity: Double, val angularVelocity: Double) {
    fun toMotorVelocity(statistics: DriveTrainStatistics): MotorVelocity {        
        val rightVelocity = (2*linearVelocity + angularVelocity*statistics.wheelDistance) / (statistics.wheelRadius * 2)
        val leftVelocity = ((2*linearVelocity)/statistics.wheelRadius) - rightVelocity
        return MotorVelocity(rightVelocity, leftVelocity)
    }
}

data class MotorVelocity(val rightVelocity: Double, val leftVelocity: Double) {
    fun toDriveTrainState(statistics: DriveTrainStatistics) = DriveTrainState(
        (statistics.wheelRadius * (rightVelocity+leftVelocity)) / 2,
        (statistics.wheelRadius * (rightVelocity-leftVelocity)) / statistics.wheelDistance
    )
}
