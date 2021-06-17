package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@TeleOp
@Disabled
public class TargetDetection extends LinearOpMode {
    OpenCvCamera webcam;
    static RotatedRect rotatedRectFitToContour;
    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "shooterCam"), cameraMonitorViewId);

        webcam.setPipeline(new TargetDetectionPipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the camera to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Also, we specify the rotation that the camera is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(864, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);


            }
        });


        telemetry.setMsTransmissionInterval(20);

        waitForStart();

        while(opModeIsActive()){
            if(rotatedRectFitToContour != null) {

                telemetry.addData("rectangle area: ", rotatedRectFitToContour.size);
                telemetry.addData("rectangle proportions", rotatedRectFitToContour.size.width / rotatedRectFitToContour.size.height);
            }
            telemetry.update();
            sleep(20);
        }
    }
    static class TargetDetectionPipeline extends OpenCvPipeline
    {
        //Image Buffers
        Mat hueMat = new Mat();
        Mat thresholdMat2 = new Mat();
        Mat finalThreshold = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();

        //Noise reduction elements
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));
        Mat dilateElement  = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6,6));

        //Colors
        static final Scalar BLUE_MIN = new Scalar(100, 66, 38);
        static final Scalar BLUE_MAX = new Scalar(130, 255, 255);
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar RED = new Scalar(255, 0, 0);

        static final int CONTOUR_LINE_THICKNESS = 2;

        enum Stage
        {
            FINAL,
            H,
            MASK,
            MASK_NR,
            CONTOURS;
        }

        Stage[] stages = Stage.values();

        //keep track of viewport stage
        int stageNum = 0;

        @Override
        public void onViewportTapped()
        {
            //change viewport stage
            int nextStageNum = stageNum + 1;
            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageNum = nextStageNum;
        }

        @Override
        public Mat processFrame(Mat input)
        {
            for(MatOfPoint contour : findContours(input))
            {
                analyzeContour(contour, input);
            }

            switch (stages[stageNum])
            {
                case H:
                {
                    return hueMat;
                }

                case FINAL:
                {
                    return input;
                }

                case MASK:
                {
                    return finalThreshold;
                }

                case MASK_NR:
                {
                    return morphedThreshold;
                }

                case CONTOURS:
                {
                    return contoursOnPlainImageMat;
                }
            }
            return input;
        }
        ArrayList<MatOfPoint> findContours(Mat input)
        {
            //Contour storage
            ArrayList<MatOfPoint> contoursList = new ArrayList<>();

            //Convert image to HSV, and Threshold Image, then extract H channel
            Imgproc.cvtColor(input, hueMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hueMat, BLUE_MIN, BLUE_MAX, thresholdMat2);
            Core.extractChannel(thresholdMat2, finalThreshold, 0);
            //Reduce noise
            morphMask(finalThreshold, morphedThreshold);

            //Find contours
            Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

            //draw contours on a separate image for staging purposes
            input.copyTo(contoursOnPlainImageMat);
            Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);

            return contoursList;
        }

        void morphMask(Mat input, Mat output)
        {
            //noise reduction through erosion and dilation

            Imgproc.erode(input, output, erodeElement);
            Imgproc.erode(output, output, erodeElement);

            Imgproc.dilate(output, output, dilateElement);
            Imgproc.dilate(output, output, dilateElement);
        }

        void analyzeContour(MatOfPoint contour, Mat input) {
            //convert contour to different format
            Point[] points = contour.toArray();
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            //do a rect fit to the contour
            rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
            if (rotatedRectFitToContour.size.height * rotatedRectFitToContour.size.width >= 1750)
            {
                drawRotatedRect(rotatedRectFitToContour, input);
            }


            //set correct rect angle
            double rotRectAngle = rotatedRectFitToContour.angle;
            if(rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height)
            {
                rotRectAngle += 90;
            }

        }

        static void drawRotatedRect(RotatedRect rect, Mat drawOn)
        {
            //draws the rotated rect

            Point[] points = new Point[4];
            rect.points(points);

            for(int i = 0; i < 4; i++)
            {
                Imgproc.line(drawOn, points[i], points[(i+1)%4], RED, 2);
            }
        }
    }

}

