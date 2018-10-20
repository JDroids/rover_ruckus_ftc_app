package main

import main.*

data class Waypoint(val x: Double, val y: Double)

interface Path {
    // t should range from 0 to 1, 1 being the end of the path, 0 being the beginning
    fun generate(t: Double): Waypoint

    val length: Double
}

interface ConstantCurvaturePath : Path {
    val curvature: Double
}