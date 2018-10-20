package org.firstinspires.ftc.teamcode.pathplanning

class LinearPath(val firstPoint: Waypoint, val secondPoint: Waypoint) : ConstantCurvaturePath {
    override val curvature = 0.0
    override val length = Math.sqrt(
        Math.pow(firstPoint.x-secondPoint.x, 2.0) +
        Math.pow(firstPoint.y-secondPoint.y, 2.0)
    )

    private val slope = (secondPoint.y-firstPoint.y) / (secondPoint.x-firstPoint.x)

    private val checkT = {t: Double ->  if (t < 0 || t > 1) throw IllegalArgumentException("A t value not between 0 and 1 was provided: $t")}

    override fun generate(t: Double): Waypoint {
        checkT(t)
        val tScaled = (t * (secondPoint.x-firstPoint.x)) + firstPoint.x

        return Waypoint(tScaled, tScaled*slope)
    }
}