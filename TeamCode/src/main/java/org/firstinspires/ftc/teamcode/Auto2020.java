/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
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
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
@Disabled
@Autonomous(name= "Auto Blue 2020")
public class Auto2020 extends LinearOpMode {
    /* Declare OpMode members. */
    OpenCvInternalCamera2 phoneCam;  // Create phone webcam object
    OpenCvCamera webcam;  // Create webcam object
    static RotatedRect rotatedRectFitToContour;
    static RotatedRect Target;
    static RotatedRect Ring;
    boolean aiming = true;
    double error = 1000;
    double output;
    double shootP = 2;
    double ringP = -2;
    double POutput;
    double minSpeed = 0;
    double shootCenter = 460;
    double shootRange = 5;
    double shootThresh = 300;
    double ringCenter = 142;
    double ringRange = 15;
    double ringThresh = 300;
    double a = -0.00278928;
    double b = 1.05409;
    double c = -126.017;
    double d = 6580.33;
    int wobbleGoalStartPos;
    int ringState = 3;
    boolean wobbleToggle = false;
    int wobblePos = 0;
    boolean isHoldingWobble = true;
    double motorSpeed;
    double DOutput;
    double D = 1.2;
    double startPoint;
    boolean firstTime = true;
    double previousError;
    ElapsedTime loopTime = new ElapsedTime();
    Hardware2021 robot = new Hardware2021();   // Use hardware map


    @Override
    public void runOpMode() {
        //start opencv webcam/ring detection pipeline
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY); //Whether to split the container vertically or horizontally
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, viewportContainerIds[0]);

        phoneCam.setPipeline(new RingDetectionPipeline());

        phoneCam.openCameraDevice();
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        //Start opencv camera
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "shooterCam"), viewportContainerIds[1]);

        webcam.setPipeline(new TargetDetectionPipeline());

        //start camera stream
        webcam.openCameraDevice();
        webcam.startStreaming(864, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);


        //set framerate
        telemetry.setMsTransmissionInterval(10);

        //set servo pos and init hardware
        robot.init(hardwareMap);
        robot.pusher.setPosition(0.666);
        robot.wobbleServo.setPosition(0.4123);
        robot.wobbleMotor.setPower(0);
        ((DcMotorEx)robot.FR).setVelocityPIDFCoefficients(30,0,0,0);
        ((DcMotorEx)robot.FL).setVelocityPIDFCoefficients(30,0,0,0);
        ((DcMotorEx)robot.BR).setVelocityPIDFCoefficients(30,0,0,0);
        ((DcMotorEx)robot.BL).setVelocityPIDFCoefficients(30,0,0,0);
        ((DcMotorEx)robot.shooter).setVelocityPIDFCoefficients(900,1.2,0,0);

        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        //wait for the game to start (driver presses PLAY)
        waitForStart();
        ((DcMotorEx)robot.shooter).setVelocity(1600);
        Ring = null;
        sleep(1000);
        if(Ring !=null) {
            if (Ring.size.height > Ring.size.width) {
                if (Ring.size.width <= 60 && Ring.size.width >= 20) {
                    ringState = 1;
                } else if (Ring.size.width >= 60) {
                    ringState = 4;
                } else {
                    ringState = 0;
                }
            }
            else
            {
                if (Ring.size.height <= 60 && Ring.size.height >= 20) {
                    ringState = 1;
                } else if (Ring.size.height >= 60) {
                    ringState = 4;
                } else {
                    ringState = 0;
                }
            }
        }
        else
        {
            ringState = 0;
        }
        telemetry.addData("case", ringState);
        telemetry.update();
        switch(ringState){
            case 0:
                loopTime.reset();
                while(aiming) {
                    aiming = aim(Target, shootCenter, shootRange, shootP, shootThresh, true);
                }
                shoot(0, false);
                aiming = true;
                firstTime = true;
                loopTime.reset();
                while(aiming) {
                    aiming = aim(Target, 570, shootRange, shootP, shootThresh, false);
                }
                drive(0.5,0.5,0.5,0.5, 1400);
                drop();
                drive(-0.75,-0.75,-0.75,-0.75, -1750);
                raise();
                aiming = true;
                firstTime = true;
                loopTime.reset();
                while(aiming) {
                    aiming = aim(Target, 737, shootRange, shootP, shootThresh, false);
                }
                rotate(-0.3,0.3,265);
                drop();
                sleep(200);
                drive(0.5,0.5,0.5,0.5,295);
                raise();
                aiming = true;
                firstTime = true;
                loopTime.reset();
                while(aiming) {
                    aiming = aim(Target, 483, shootRange, shootP, shootThresh, false);
                }
                drive(0.75,0.75,0.75,0.75,1650);
                drop();
                break;
           //case with one ring
            case 1:
                loopTime.reset();
                while(aiming) {
                    aiming = aim(Target, shootCenter, shootRange, shootP, shootThresh, true);
                }
                shoot(0, true);
                aiming = true;
                loopTime.reset();
                firstTime = true;
                while(aiming) {
                    aiming = aim(Ring, ringCenter, ringRange, ringP, ringThresh, false);
                }
                robot.intake.setPower(1);
                drive(1,1,1,1, 500);
                aiming = true;
                loopTime.reset();
                firstTime = true;
                while(aiming) {
                    aiming = aim(Target, shootCenter, shootRange, shootP, shootThresh, true);
                }
                ((DcMotorEx)robot.shooter).setVelocity(1600);
                sleep(800);
                shoot(2, false);
                robot.intake.setPower(0);
                aiming = true;
                loopTime.reset();
                firstTime = true;
                while(aiming) {
                    aiming = aim(Target, 338, shootRange, shootP, shootThresh, false);
                }
                drive(1,1,1,1, 1200);
                drop();
                sleep(100);
                drive(-1,-1,-1,-1,-2200);
                aiming = true;
                firstTime = true;
                loopTime.reset();
                while(aiming) {
                    aiming = aim(Target, 737, shootRange, shootP, shootThresh, false);
                }
                rotate(-0.4,0.4,300);
                drive(0.5,0.5,0.5,0.5,270);
                raise();
                aiming = true;
                firstTime = true;
                loopTime.reset();
                while(aiming) {
                    aiming = aim(Target, 280, shootRange, shootP, shootThresh, false);
                }
                drive(1,1,1,1, 1850);
                drop();
                break;
            case 3:
                telemetry.addData("Say", "die");
                break;
            case 4:
                loopTime.reset();
                while(aiming) {
                    aiming = aim(Target, shootCenter, shootRange, shootP, shootThresh, true);
                }
                shoot(0, true);
                aiming = true;
                loopTime.reset();
                firstTime = true;
                while(aiming) {
                    aiming = aim(Ring, ringCenter, ringRange, ringP, ringThresh, false);
                }
                robot.intake.setPower(1);
                drive(0.75,0.75,0.75,0.75, 300);
                sleep(300);
                drive(-0.25,-0.25,-0.25,-0.25,-50);
                drive(0.25,0.25,0.25,0.25, 150);
                sleep(200);
                drive(-0.25,-0.25,-0.25,-0.25,-50);
                drive(0.25,0.25,0.25,0.25, 200);
                drive(-0.5,-0.5,-0.5,-0.5,-200);
                aiming = true;
                firstTime = true;
                loopTime.reset();
                while(aiming) {
                    aiming = aim(Target, shootCenter, shootRange, shootP, shootThresh, true);
                }
                shoot(0, true);
                robot.intake.setPower(0);
                aiming = true;
                loopTime.reset();
                firstTime = true;
                while(aiming) {
                    aiming = aim(Ring, ringCenter, ringRange, ringP, ringThresh, false);
                }
                robot.intake.setPower(1);
                drive(1,1,1,1, 300);
                aiming = true;
                loopTime.reset();
                firstTime = true;
                while(aiming) {
                    aiming = aim(Target, shootCenter, shootRange, shootP, shootThresh, true);
                }
                sleep(1200);
                shoot(2, false);
                robot.intake.setPower(0);
                aiming = true;
                loopTime.reset();
                firstTime = true;
                while(aiming) {
                    aiming = aim(Target, 515, shootRange, shootP, shootThresh, false);
                }
                drive(1,1,1,1, 1650);
                drop();
                drive(-1,-1,-1,-1, -800);
                break;
        }




        /*while(opModeIsActive()) {
            if (Target != null) {
                telemetry.addData("Target Width", Target.size.width);
                telemetry.addData("Target Height", Target.size.height);
                telemetry.addData("Target Center y", Target.center.y);
            }
            if (Ring != null) {
                telemetry.addData("Ring Width", Ring.size.width);
                telemetry.addData("Ring Height", Ring.size.height);
                telemetry.addData("Ring Center y", Ring.center.y);
            }
            if (Ring == null)
            {
                telemetry.addData("Say", "Ring Not Detected");
            }
            if(Target == null)
            {
                telemetry.addData("Say", "Target Not Detected");
            }
            telemetry.addData("Say", "Outside of ifs");
            telemetry.update();
        }
        */

        /*Case 0

        drive(0.385, 0.5, 0.385, 0.5);
        sleep(5700);
        drive(0, 0, 0, 0);

         */


        /*Case 1

        drive(0.5, 0.53, 0.5, 0.53);
        sleep(6000);
        drive(-0.5, -0.53, -0.5, -0.53);
        sleep(1200);
        drive(0, 0, 0, 0);

         */

        /*Case 4

        drive(0.5, 0.54, 0.5, 0.54);
        sleep(8400);
        drive(-0.5,-0.52,-0.5,-0.52);
        sleep(3500);
        drive(0, 0, 0, 0);

         */

    }

    static class RingDetectionPipeline extends OpenCvPipeline
    {
        //Image Buffers
        Mat hueMat = new Mat();
        Mat thresholdMat2 = new Mat();
        Mat finalThreshold = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();

        //Threshold values

        //Noise reduction elements
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));
        Mat dilateElement  = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6,6));

        //Colors
        static final Scalar RING_MIN = new Scalar(0, 150, 70);
        static final Scalar RING_MAX = new Scalar(25, 255, 255);
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar BLUE = new Scalar(0, 0, 255);

        static final int CONTOUR_LINE_THICKNESS = 2;

        enum Stage
        {
            FINAL,
            Cb,
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
                case Cb:
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

            //Gamma adjust image, convert image to HSV, and Threshold Image, then extract H channel
            Mat lookUpTable = new Mat(1, 256, CvType.CV_8U);
            byte[] lookUpTableData = new byte[(int) (lookUpTable.total()*lookUpTable.channels())];
            for (int i = 0; i < lookUpTable.cols(); i++) {
                lookUpTableData[i] = saturate(Math.pow(i / 255.0, 0.4) * 255.0);
            }
            lookUpTable.put(0, 0, lookUpTableData);
            Core.LUT(input, lookUpTable, input);
            Imgproc.cvtColor(input, hueMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hueMat, RING_MIN, RING_MAX, thresholdMat2);
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
        private byte saturate(double val)
        {
            int iVal = (int) Math.round(val);
            iVal = iVal > 255 ? 255 : (iVal < 0 ? 0 : iVal);
            return (byte) iVal;
        }

        void morphMask(Mat input, Mat output)
        {
            //noise reduction through erosion and dilation

            Imgproc.erode(input, output, erodeElement);
            Imgproc.erode(output, output, erodeElement);

            Imgproc.dilate(output, output, dilateElement);
            Imgproc.dilate(output, output, dilateElement);
        }

        void analyzeContour(MatOfPoint contour, Mat input)
        {
            //convert contour to different format
            Point[] points = contour.toArray();
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            //do a rect fit to the contour
            rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
            if (rotatedRectFitToContour.size.area() >= 300) {
                drawRotatedRect(rotatedRectFitToContour, input);
                Ring = rotatedRectFitToContour;
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

    static class TargetDetectionPipeline extends OpenCvPipeline {
        //Image Buffers
        Mat hueMat = new Mat();
        Mat thresholdMat2 = new Mat();
        Mat finalThreshold = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();

        //Noise reduction elements
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

        //Colors
        static final Scalar BLUE_MIN = new Scalar(100, 45, 90);
        static final Scalar BLUE_MAX = new Scalar(130, 255, 255);
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar RED = new Scalar(255, 0, 0);

        static final int CONTOUR_LINE_THICKNESS = 2;

        enum Stage {
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
        public void onViewportTapped() {
            //change viewport stage
            int nextStageNum = stageNum + 1;
            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageNum = nextStageNum;
        }

        @Override
        public Mat processFrame(Mat input) {
            for (MatOfPoint contour : findContours(input)) {
                analyzeContour(contour, input);
            }

            switch (stages[stageNum]) {
                case H: {
                    return hueMat;
                }

                case FINAL: {
                    return input;
                }

                case MASK: {
                    return finalThreshold;
                }

                case MASK_NR: {
                    return morphedThreshold;
                }

                case CONTOURS: {
                    return contoursOnPlainImageMat;
                }
            }
            return input;
        }

        ArrayList<MatOfPoint> findContours(Mat input) {
            //Contour storage
            ArrayList<MatOfPoint> contoursList = new ArrayList<>();

            //Gamma adjust image, convert image to HSV, and Threshold Image, then extract H channel
            Mat lookUpTable = new Mat(1, 256, CvType.CV_8U);
            byte[] lookUpTableData = new byte[(int) (lookUpTable.total() * lookUpTable.channels())];
            for (int i = 0; i < lookUpTable.cols(); i++) {
                lookUpTableData[i] = saturate(Math.pow(i / 255.0, 0.65) * 255.0);
            }
            lookUpTable.put(0, 0, lookUpTableData);
            Core.LUT(input, lookUpTable, input);
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

        void morphMask(Mat input, Mat output) {
            //noise reduction through erosion and dilation

            Imgproc.erode(input, output, erodeElement);
            Imgproc.erode(output, output, erodeElement);

            Imgproc.dilate(output, output, dilateElement);
            Imgproc.dilate(output, output, dilateElement);
        }

        void analyzeContour(MatOfPoint contour, Mat input) {
            //convert contour to different format
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            //do a rect fit to the contour
            rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
            if (rotatedRectFitToContour.size.area() >= 5000 && rotatedRectFitToContour.size.area() <= 50000 && rotatedRectFitToContour.size.width / rotatedRectFitToContour.size.height >= 0.6 && rotatedRectFitToContour.size.width / rotatedRectFitToContour.size.height <= 1.75) {
                drawRotatedRect(rotatedRectFitToContour, input);
                Target = rotatedRectFitToContour;
            }

        }

        static void drawRotatedRect(RotatedRect rect, Mat drawOn) {
            //draws the rotated rect

            Point[] points = new Point[4];
            rect.points(points);

            for (int i = 0; i < 4; i++) {
                Imgproc.line(drawOn, points[i], points[(i + 1) % 4], RED, 2);
            }
        }

        private byte saturate(double val) {
            int iVal = (int) Math.round(val);
            iVal = iVal > 255 ? 255 : (iVal < 0 ? 0 : iVal);
            return (byte) iVal;
        }

    }

    static double threshold(double value, double max, double min) {
        if (value > max) {
            value = max;
        } else if (value < min) {
            value = min;
        } else {
            value = value;
        }
        return value;
    }

    public void drive(double FLPower, double FRPower, double BLPower, double BRPower, int distance)
    {
        robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int dist = 0;
        ((DcMotorEx)robot.FL).setVelocity(2000*FLPower);
        ((DcMotorEx)robot.FR).setVelocity(2000*FRPower);
        ((DcMotorEx)robot.BL).setVelocity(2000*BLPower);
        ((DcMotorEx)robot.BR).setVelocity(2000*BRPower);

        while(opModeIsActive()&&dist<distance && distance >= 0)
             dist=(((DcMotorEx)robot.FL).getCurrentPosition()+((DcMotorEx)robot.BR).getCurrentPosition()+((DcMotorEx)robot.BL).getCurrentPosition()+((DcMotorEx)robot.FR).getCurrentPosition())/4;
        while(opModeIsActive()&&dist>distance && distance <= 0)
            dist=(((DcMotorEx)robot.FL).getCurrentPosition()+((DcMotorEx)robot.BR).getCurrentPosition()+((DcMotorEx)robot.BL).getCurrentPosition()+((DcMotorEx)robot.FR).getCurrentPosition())/4;

        ((DcMotorEx)robot.FL).setVelocity(0);
        ((DcMotorEx)robot.FR).setVelocity(0);
        ((DcMotorEx)robot.BL).setVelocity(0);
        ((DcMotorEx)robot.BR).setVelocity(0);
    }

    public boolean aim (RotatedRect aimAt, double setPoint, double range, double P, double thresh, boolean shoot) {
        if(firstTime) {
            ((DcMotorEx)robot.FL).setVelocityPIDFCoefficients(30,10,2,0);
            ((DcMotorEx)robot.FR).setVelocityPIDFCoefficients(30,10,2,0);
            ((DcMotorEx)robot.BL).setVelocityPIDFCoefficients(30,10,2,0);
            ((DcMotorEx)robot.BR).setVelocityPIDFCoefficients(30,10,2,0);
            startPoint = aimAt.center.y;
            previousError = setPoint - startPoint;
            firstTime = false;
        }
        sleep(75-((int)loopTime.milliseconds()));
        startPoint = aimAt.center.y;
        if(shoot)
        {
            double distance;
            if (aimAt.size.height < aimAt.size.width)
            {
                distance = aimAt.size.height;
            }
            else
            {
                distance = aimAt.size.width;
            }
            double targetSpeed = (a * (distance * distance * distance)) + (b * (distance*distance)) + (c*distance) + d;
            ((DcMotorEx) robot.shooter).setVelocity(targetSpeed);
        }
        error = setPoint - startPoint;
        if (error >= range || error <= -range) {
            POutput = error * P;
            DOutput = D*(error-previousError);
            if (POutput <= minSpeed && POutput >= 0) {
                output = minSpeed;
            } else if (POutput <= 0 && POutput >= -minSpeed) {
                output = -minSpeed;
            } else {
                output = threshold(POutput, thresh, -thresh);
            }
            output += (DOutput*P);
            previousError = error;
            ((DcMotorEx) robot.FL).setVelocity(-output);
            ((DcMotorEx) robot.FR).setVelocity(output);
            ((DcMotorEx) robot.BL).setVelocity(-output);
            ((DcMotorEx) robot.BR).setVelocity(output);
            telemetry.addData("Say", "In Loop");
            telemetry.addData("error", error);
            telemetry.addData("D Out", DOutput);
            telemetry.addData("output power", output);
            telemetry.addData("center pos", startPoint);
            loopTime.reset();
        }
        if (error <= range && error >= -range && aiming) {
            sleep(200);
            error = setPoint - startPoint;
            if(error <= range && error >= -range && aiming) {
                ((DcMotorEx) robot.FR).setVelocityPIDFCoefficients(30, 0, 0, 0);
                ((DcMotorEx) robot.FL).setVelocityPIDFCoefficients(30, 0, 0, 0);
                ((DcMotorEx) robot.BR).setVelocityPIDFCoefficients(30, 0, 0, 0);
                ((DcMotorEx) robot.BL).setVelocityPIDFCoefficients(30, 0, 0, 0);
                ((DcMotorEx) robot.FL).setVelocity(0);
                ((DcMotorEx) robot.FR).setVelocity(0);
                ((DcMotorEx) robot.BL).setVelocity(0);
                ((DcMotorEx) robot.BR).setVelocity(0);
                return false;
            }
            else
            {
                loopTime.reset();
                return true;
            }
        }
        return true;
    }
    public void shoot (int i, boolean keepShooting) {
        double distance;
        if (Target.size.height < Target.size.width)
        {
            distance = Target.size.height;
        }
        else
        {
            distance = Target.size.width;
        }
        double targetSpeed = (a * (distance * distance * distance)) + (b * (distance*distance)) + (c*distance) + d;
        ((DcMotorEx) robot.shooter).setVelocity(targetSpeed);
        while (motorSpeed < targetSpeed*0.989 && opModeIsActive() || motorSpeed >= (targetSpeed*1.011) && opModeIsActive())
        {
            motorSpeed = ((DcMotorEx) robot.shooter).getVelocity();

        }
        aiming = false;
        motorSpeed = ((DcMotorEx) robot.shooter).getVelocity();
        for (i = i; i <= 2; i++) {
            while (motorSpeed < targetSpeed*0.989 && opModeIsActive() || motorSpeed > (targetSpeed+1.011) && opModeIsActive())
            {
                motorSpeed = ((DcMotorEx) robot.shooter).getVelocity();

            }
            robot.pusher.setPosition(1);
            sleep(300);
            robot.pusher.setPosition(0.666);
            sleep(300);
        }
        if(keepShooting)
        {
            ((DcMotorEx)robot.shooter).setVelocity(motorSpeed);
        }
        else
        {
            ((DcMotorEx) robot.shooter).setVelocity(0);
        }
    }
    public void drop()
    {
        robot.wobbleMotor.setPower(0.5);
        wobblePos = ((DcMotorEx)robot.wobbleMotor).getCurrentPosition();
        while(wobblePos<=650 && opModeIsActive())
        {
            wobblePos = ((DcMotorEx)robot.wobbleMotor).getCurrentPosition();
        }
        robot.wobbleMotor.setPower(0);
        robot.wobbleServo.setPosition(0);
    }
    public void raise()
    {
        robot.wobbleServo.setPosition(0.4065);
        sleep(700);
        robot.wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.wobbleMotor.setPower(-0.5);
        sleep(100);
        while(((DcMotorEx)robot.wobbleMotor).getVelocity()<=-100 && opModeIsActive())
        {
            sleep(1);
        }
        robot.wobbleMotor.setPower(0);
        sleep(100);
        robot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotate(double left, double right, int position)
    {
        robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int dist = 0;
        ((DcMotorEx)robot.FL).setVelocity(2000*left);
        ((DcMotorEx)robot.FR).setVelocity(2000*right);
        ((DcMotorEx)robot.BL).setVelocity(2000*left);
        ((DcMotorEx)robot.BR).setVelocity(2000*right);

        while(opModeIsActive()&&dist<position && position >= 0)
            dist=(Math.abs(((DcMotorEx)robot.FL).getCurrentPosition())+Math.abs(((DcMotorEx)robot.BR).getCurrentPosition())+Math.abs(((DcMotorEx)robot.BL).getCurrentPosition())+(Math.abs(((DcMotorEx)robot.FR).getCurrentPosition())))/4;
        while(opModeIsActive()&&dist>position && position <= 0)
            dist=(Math.abs(((DcMotorEx)robot.FL).getCurrentPosition())+Math.abs(((DcMotorEx)robot.BR).getCurrentPosition())+Math.abs(((DcMotorEx)robot.BL).getCurrentPosition())+(Math.abs(((DcMotorEx)robot.FR).getCurrentPosition())))/4;
        ((DcMotorEx)robot.FL).setVelocity(0);
        ((DcMotorEx)robot.FR).setVelocity(0);
        ((DcMotorEx)robot.BL).setVelocity(0);
        ((DcMotorEx)robot.BR).setVelocity(0);
    }
}
