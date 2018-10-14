/*package org.firstinspires.ftc.teamcode

import org.corningrobotics.enderbots.endercv.OpenCVPipeline
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.opencv.core.Mat



class SamplingVision : OpenCVPipeline {
    enum class GoldPosition {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }

    var goldPosition: GoldPosition = GoldPosition.NONE

    private val bgr = Mat()
    private val lab = Mat()

    private val yellowThresholded = Mat()
    private val whiteThresholded = Mat()

    private val yellowContours = ArrayList<MatOfPoint>()
    private val whiteContours = ArrayList<MatOfPoint>()

    override fun processFrame(rgba: Mat?, gray: Mat?): Mat {
        Imgproc.cvtColor(rgba, bgr, Imgproc.COLOR_RGBA2BGR)

        Imgproc.blur(lab, lab, Size(3.0, 3.0))

        Imgproc.cvtColor(bgr, lab, Imgproc.COLOR_BGR2Lab)

        Core.inRange(lab, Scalar(120.0, 100.0, 150.0), Scalar(255.0, 154.0, 255.0), yellowThresholded)
        Core.inRange(lab, Scalar(170.0, 97.0, 97.0), Scalar(255.0, 157.0, 157.0), whiteThresholded)

        yellowContours.clear()
        whiteContours.clear()

        Imgproc.findContours(yellowThresholded, yellowContours, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)
        Imgproc.findContours(whiteThresholded, whiteContours, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)

        yellowContours.sortBy { point -> Imgproc.contourArea(point) }
        whiteContours.sortBy { point -> Imgproc.contourArea(point) }

        val frameArea = lab.height() * lab.width()

        val minArea = frameArea / 4000
        val maxArea = frameArea / 100

        var yellowCenterX: Double = 0.0

        for (contour in yellowContours) {
            val boundingRect = Imgproc.boundingRect(contour)
            val boundingArea = boundingRect.area()
            val contourArea = Imgproc.contourArea(contour)

            val lengthOverHeight = boundingRect.width / boundingRect.height

            if (contourArea <= maxArea && contourArea >= minArea && )
        }
    }
}*/