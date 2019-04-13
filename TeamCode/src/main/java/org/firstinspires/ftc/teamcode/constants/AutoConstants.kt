package org.firstinspires.ftc.teamcode.constants

import com.acmerobotics.dashboard.config.Config

@Config
object SharedAutoConstants {
    @JvmField var MARKER_INIT_POS = 0.65
    @JvmField var MARKER_DROP_POS = 0.0

    @JvmField var LEFT_SAMPLE_ANGLE = 215.0
    @JvmField var RIGHT_SAMPLE_ANGLE = 155.0

    @JvmField var GET_OFF_HOOK_DISTANCE = 0.3
    @JvmField var GET_OFF_HOOK_POWER = 0.3

    @JvmField var HIT_SAMPLE_DISTANCE = 2.4
    @JvmField var HIT_SAMPLE_POWER = 0.3

    @JvmField var BACK_FROM_SAMPLE_DISTANCE = -1.9
    @JvmField var BACK_FROM_SAMPLE_POWER = 0.5

    @JvmField var TURN_TO_WALL_ANGLE = 242.5

    @JvmField var TARGET_DISTANCE_FROM_WALL = 4.0
}

@Config
object CraterAutoConstants {
    @JvmField var TURN_TOWARDS_DEPOT_ANGLE = 305.0

    @JvmField var DRIVE_TO_DEPOT_DISTANCE = 3.5
    @JvmField var DRIVE_TO_DEPOT_POWER = 0.6

    @JvmField var DRIVE_TO_CRATER_DISTANCE = -5.1
    @JvmField var DRIVE_TO_CRATER_POWER = 0.25

    @JvmField var TURN_BACK_OF_ROBOT_TOWARDS_WALL_TO_AVOID_SAMPLE_FIELD_ANGLE = 320.0
}

@Config
object DepotAutoConstants {
    @JvmField var TURN_TOWARDS_DEPOT_ANGLE = 135.0

    @JvmField var DRIVE_TO_DEPOT_DISTANCE = 1.4
    @JvmField var DRIVE_TO_DEPOT_POWER = 0.2

    @JvmField var DRIVE_TO_CRATER_DISTANCE = -3.4
    @JvmField var DRIVE_TO_CRATER_POWER = 0.25
}