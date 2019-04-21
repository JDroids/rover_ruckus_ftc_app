package org.firstinspires.ftc.teamcode.constants

import com.acmerobotics.dashboard.config.Config

@Config
object SharedAutoConstants {
    @JvmField var MARKER_INIT_POS = 0.65
    @JvmField var MARKER_DROP_POS = 0.0

    @JvmField var LEFT_SAMPLE_ANGLE = 160.0
    //@JvmField var CENTER_SAMPLE_ANGLE = 180.0
    @JvmField var RIGHT_SAMPLE_ANGLE = 220.0

    @JvmField var GET_OFF_HOOK_DISTANCE = 0.4
    @JvmField var GET_OFF_HOOK_POWER = 0.3

    @JvmField var HIT_SAMPLE_DISTANCE = 2.4
    @JvmField var HIT_SAMPLE_POWER = 0.3

    @JvmField var BACK_FROM_SAMPLE_DISTANCE = -2.1
    @JvmField var BACK_FROM_SAMPLE_POWER = 0.5

    @JvmField var TURN_TO_WALL_ANGLE = 260.0

    @JvmField var TO_WALL_DISTANCE_IN_INCHES = 3.5

    @JvmField var TURN_BACK_OF_ROBOT_TOWARDS_WALL_TO_AVOID_SAMPLE_FIELD_ANGLE = 320.0
}

@Config
object CraterAutoConstants {
    @JvmField var TURN_TOWARDS_DEPOT_ANGLE = 320.0

    @JvmField var DRIVE_TO_DEPOT_DISTANCE = 3.5
    @JvmField var DRIVE_TO_DEPOT_POWER = 0.6

    @JvmField var DRIVE_TO_CRATER_DISTANCE = -5.8
    @JvmField var DRIVE_TO_CRATER_POWER = 0.25
}

@Config
object DepotAutoConstants {
    @JvmField var TURN_TOWARDS_DEPOT_ANGLE = 150.0

    @JvmField var DRIVE_TO_DEPOT_DISTANCE = 4.3
    @JvmField var DRIVE_TO_DEPOT_POWER = 0.5

    @JvmField var DRIVE_TO_CRATER_DISTANCE = 5.1
    @JvmField var DRIVE_TO_CRATER_POWER = 0.3
}