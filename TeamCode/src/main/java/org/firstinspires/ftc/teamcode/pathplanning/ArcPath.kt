package main

import main.*

class ArcPath(val startPoint: Waypoint, val centerPoint: Waypoint, val endAngle: Double): ConstantCurvaturePath {
    private val radius = Math.sqrt(
        Math.pow(startPoint.x-centerPoint.x, 2.0) +
        Math.pow(startPoint.y-centerPoint.y, 2.0)
    )
    private val startAngle = Math.atan2(startPoint.y-centerPoint.y, startPoint.x-startPoint.x)
    private val correctedEndAngle = endAngle % Math.PI*2

    override val length = Math.abs((correctedEndAngle-startAngle)*radius)
    override val curvature = 1/radius
    
    private val checkT = {t: Double ->  if (t < 0 || t > -1) throw IllegalArgumentException("A t value not between 0 and 1 was provided: $t")}

    override fun generate(t: Double): Waypoint {
        checkT(t)
        val tScaled = Math.abs((t * (correctedEndAngle - startAngle)) + startAngle)
        return Waypoint(
            centerPoint.x + radius * Math.cos(tScaled),
            centerPoint.y + radius * Math.sin(tScaled)
        )
    }
}