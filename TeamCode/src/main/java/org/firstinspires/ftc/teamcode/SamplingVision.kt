package org.firstinspires.ftc.teamcode

/*
import android.util.Log
import com.acmerobotics.dashboard.config.Config
import org.corningrobotics.enderbots.endercv.OpenCVPipeline
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.opencv.core.Mat
import java.lang.NullPointerException
*/

class SamplingVision /*: OpenCVPipeline()*/ {
    /*@Config
    object VisionConstants {
        @JvmField var yellowMinL = 120.0
        @JvmField var yellowMaxL = 255.0
        @JvmField var yellowMinA = 100.0
        @JvmField var yellowMaxA = 154.0
        @JvmField var yellowMinB = 150.0
        @JvmField var yellowMaxB = 255.0
    }*/

    enum class GoldPosition {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }
    /*
    var goldPosition: GoldPosition = GoldPosition.NONE
        private set

    private var bgr = Mat()
    private var lab = Mat()

    private var yellowThresholded = Mat()
    private var whiteThresholded = Mat()

    private var yellowContours = ArrayList<MatOfPoint>()
    private var whiteContours = ArrayList<MatOfPoint>()

    override fun processFrame(rgba: Mat?, gray: Mat?): Mat {
        Imgproc.cvtColor(rgba, bgr, Imgproc.COLOR_RGBA2BGR)

        Imgproc.cvtColor(bgr, lab, Imgproc.COLOR_BGR2Lab)

        Core.inRange(lab,
                Scalar(VisionConstants.yellowMinL, VisionConstants.yellowMinA, VisionConstants.yellowMinB),
                Scalar(VisionConstants.yellowMaxL, VisionConstants.yellowMaxA, VisionConstants.yellowMaxB),
                yellowThresholded)
        Core.inRange(lab, Scalar(170.0, 97.0, 97.0), Scalar(255.0, 157.0, 157.0), whiteThresholded)

        yellowContours.clear()
        whiteContours.clear()

        Imgproc.findContours(yellowThresholded, yellowContours, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)
        Imgproc.findContours(whiteThresholded, whiteContours, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)

        val returnFrame = yellowThresholded
        Imgproc.cvtColor(returnFrame, returnFrame, Imgproc.COLOR_GRAY2BGR)

        yellowContours.sortBy { point -> Imgproc.contourArea(point) }
        whiteContours.sortBy { point -> Imgproc.contourArea(point) }

        val frameArea = lab.height() * lab.width()

        val minArea = frameArea / 4000
        val maxArea = frameArea / 100

        var yellowCenterX = -1

        for (contour in yellowContours) {
            val boundingRect = Imgproc.boundingRect(contour)
            val boundingArea = boundingRect.area()
            val contourArea = Imgproc.contourArea(contour)

            val widthOverHeight = boundingRect.width / boundingRect.height

            if (contourArea <= maxArea && contourArea >= minArea &&
                    contourArea / boundingArea >= 0.7 && widthOverHeight >= 0.8 &&
                    widthOverHeight <= 1.2) {
                yellowCenterX = boundingRect.x + (boundingRect.width / 2)

                Imgproc.rectangle(returnFrame,
                        Point(boundingRect.x.toDouble(), boundingRect.y.toDouble()),
                        Point(boundingRect.x+boundingRect.width.toDouble(),
                                boundingRect.x+boundingRect.height.toDouble()),
                        Scalar(0.0, 255.0, 0.0),2)

                break
            }
        }

        val whiteCenterXs = IntArray(2) {_ -> -1}
        var foundContours = 0

        for (contour in whiteContours) {
            if (foundContours >= 2) {
                break
            }

            val boundingRect = Imgproc.boundingRect(contour)
            val boundingArea = boundingRect.area()
            val contourArea = Imgproc.contourArea(contour)

            val widthOverHeight = boundingRect.width / boundingRect.height

            if (contourArea <= maxArea && contourArea >= minArea &&
                    contourArea / boundingArea >= 0.6 && widthOverHeight >= 0.8 &&
                    widthOverHeight <= 1.7) {
                whiteCenterXs[foundContours] = boundingRect.x + (boundingRect.width / 2)

                Imgproc.rectangle(returnFrame,
                        Point(boundingRect.x.toDouble(), boundingRect.y.toDouble()),
                        Point(boundingRect.x+boundingRect.width.toDouble(),
                                boundingRect.x+boundingRect.height.toDouble()),
                        Scalar(0.0, 0.0, 255.0),2)

                ++foundContours
            }

            whiteCenterXs.sort()

            goldPosition = if (yellowCenterX != -1 && whiteCenterXs[0] != -1 && whiteCenterXs[1] != -1) {
                when {
                    yellowCenterX > whiteCenterXs[1] -> GoldPosition.RIGHT
                    yellowCenterX > whiteCenterXs[0] -> GoldPosition.CENTER
                    else -> GoldPosition.LEFT
                }
            }
            else {
                 GoldPosition.NONE
            }
        }

        //bgr.release()
        //lab.release()
        //yellowThresholded.release()
        //+whiteThresholded.release()

        return returnFrame
    }*/
}