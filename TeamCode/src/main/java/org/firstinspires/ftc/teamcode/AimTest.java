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


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
@Disabled
@TeleOp(name= "Aiming Test")
public class AimTest extends LinearOpMode {
    //create webcam and variables for target detection
    OpenCvCamera webcam;
    static RotatedRect rotatedRectFitToContour;
    static RotatedRect Target;
    boolean aiming = false;
    int wobblePos = 0;
    boolean isHoldingWobble = false;



    //create odometry coordinate object
    /* Declare OpMode members. */
    Hardware2021 robot = new Hardware2021();   // Use hardware map
    @Override
    public void runOpMode() throws InterruptedException {

        //drive variables
        double FrontLeft;
        double FrontRight;
        double RearLeft;
        double RearRight;
        double r;
        double robotAngle;
        double rightX;
        boolean shoot = false;
        double error = 1000;
        double output;
        double P = 1.5;
        double POutput;
        double minSpeed = 70;
        double endPoint = 470;
        double range = 5;
        double shooterSpeed = 1000;
        double a = 0.294555;
        double b = -60.2387;
        double c = 4725.42;
        boolean wobbleToggle = false;
        double servoSetting = 0.1;
        double Thresh = 600;
        double motorP = 20;
        double motorI = 8;
        double motorD = 2;
        double launcherP = 750;
        double motorF = 0.7;
        double launcherD = 0;
        double previousError = 0;
        double loopWait;
        boolean firstTime = true;
        ElapsedTime loopTime = new ElapsedTime();
        double startPoint;
        double D = 1.2;
        int topSpeed=2000;
        double powerMultiplier=1;
        double DOutput = 0;

        //Start opencv camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "shooterCam"), cameraMonitorViewId);

        webcam.setPipeline(new TargetDetectionPipeline());

        //start camera stream
        webcam.openCameraDevice();
        webcam.startStreaming(864, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);


        //set framerate
        telemetry.setMsTransmissionInterval(30);

        //init hardware and set servo pos
        robot.init(hardwareMap);
        robot.pusher.setPosition(0.666);
        robot.wobbleServo.setPosition(0.2713);
        ((DcMotorEx)robot.wobbleMotor).setTargetPosition(-685);
        //send telemetry messge to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        //wait for the game to start (driver presses PLAY)
        waitForStart();
        raise();

        //run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double motorSpeed = ((DcMotorEx) robot.shooter).getVelocity();
            if (gamepad1.dpad_up) {
                P += 0.1;
                sleep(100);
            }
            if (gamepad1.dpad_down) {
                P -= 0.1;
                sleep(100);
            }
            if (gamepad1.dpad_right) {
                Thresh += 100;
                sleep(100);
            }
            if (gamepad1.dpad_left) {
                Thresh -= 100;
                sleep(100);
            }
            if (gamepad1.b)
            {
                minSpeed += 10;
                sleep(100);
            }
            if(gamepad1.x)
            {
                minSpeed -= 10;
                sleep(100);
            }
            if(gamepad1.right_bumper)
            {
                range += 1;
                sleep(100);
            }
            if(gamepad1.left_bumper)
            {
                range -= 1;
                sleep(100);
            }
            if(gamepad1.right_trigger>=0.9)
            {
                D += 0.1;
                sleep(100);
            }
            if(gamepad1.left_trigger>=0.9)
            {
                D -= 0.1;
                sleep(100);
            }
            if(gamepad2.dpad_up)
            {
                motorP += 10;
                sleep(100);
            }
            if(gamepad2.dpad_down)
            {
                motorP -= 10;
                sleep(100);
            }
            if(gamepad2.dpad_right)
            {
                motorI += 0.1;
                sleep(100);
            }
            if(gamepad2.dpad_left)
            {
                motorI -= 0.1;
                sleep(100);
            }
            if(gamepad2.b)
            {
                motorD += 0.1;
                sleep(100);
            }
            if(gamepad2.x)
            {
                motorD -= 0.1;
                sleep(100);
            }
            if(gamepad2.right_bumper)
            {
                launcherP += 10;
                sleep(100);
            }
            if(gamepad2.left_bumper)
            {
                launcherP -= 10;
                sleep(100);
            }
            if(gamepad2.a)
            {
                motorF -= 0.1;
                sleep(100);
            }
            if(gamepad2.y)
            {
                motorF += 0.1;
                sleep(100);
            }
            if(gamepad2.right_stick_button)
            {
                launcherD += 0.1;
                sleep(100);
            }
            if(gamepad2.left_stick_button)
            {
                launcherD -= 0.1;
                sleep(100);
            }
            if(gamepad2.right_trigger >= 0.5)
            {
                ((DcMotorEx)robot.shooter).setVelocity(1700);
            }
            if(gamepad2.left_trigger >= 0.5)
            {
                ((DcMotorEx)robot.shooter).setVelocity(0);
            }
            if(gamepad2.start)
            {
                ((DcMotorEx)robot.FL).setVelocityPIDFCoefficients(motorP, motorI, motorD, motorF);
                ((DcMotorEx)robot.BL).setVelocityPIDFCoefficients(motorP, motorI, motorD, motorF);
                ((DcMotorEx)robot.FR).setVelocityPIDFCoefficients(motorP, motorI, motorD, motorF);
                ((DcMotorEx)robot.BR).setVelocityPIDFCoefficients(motorP, motorI, motorD, motorF);
                ((DcMotorEx)robot.shooter).setVelocityPIDFCoefficients(750, 0.7, 0, 0);
                sleep(100);
            }



            //output target size and proportions
            if (Target != null) {

                telemetry.addData("Set Speed", shooterSpeed);
                telemetry.addData("Width", Target.size.width);
                telemetry.addData("Height", Target.size.height);
                telemetry.addData("Center y", Target.center.y);
            }
            telemetry.addData("thresh value", Thresh);
            telemetry.addData("P value", P);
            telemetry.addData("D value", D);
            telemetry.addData("Min Speed", minSpeed);
            telemetry.addData("range", range);
            telemetry.addData("Motor PID", ((DcMotorEx)robot.FL).getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("Launcher PID", ((DcMotorEx)robot.shooter).getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("Motor Speed", motorSpeed);



            //do calculations to convert stick position to motor power
            if (aiming == false) {
                r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                rightX = -gamepad1.right_stick_x * .75;
                FrontLeft = r * Math.cos(robotAngle) - rightX;
                FrontRight = r * Math.sin(robotAngle) + rightX;
                RearLeft = r * Math.sin(robotAngle) - rightX;
                RearRight = r * Math.cos(robotAngle) + rightX;

                //set values to motor powers
                ((DcMotorEx) robot.FL).setVelocity(FrontLeft * topSpeed * powerMultiplier);
                ((DcMotorEx) robot.FR).setVelocity(FrontRight * topSpeed * powerMultiplier);
                ((DcMotorEx) robot.BR).setVelocity(RearRight * topSpeed * powerMultiplier);
                ((DcMotorEx) robot.BL).setVelocity(RearLeft * topSpeed * powerMultiplier);
            }


            if (aiming) {
                if(firstTime) {
                    startPoint = Target.center.y;
                    previousError = endPoint - startPoint;
                    firstTime = false;
                }
                    sleep(75-((int)loopTime.milliseconds()));
                    startPoint = Target.center.y;
                    error = endPoint - startPoint;
                    if (error >= range || error <= -range) {
                        POutput = error * P;
                        DOutput = D*(error-previousError);
                        if (POutput <= minSpeed && POutput >= 0) {
                            output = minSpeed;
                        } else if (POutput <= 0 && POutput >= -minSpeed) {
                            output = -minSpeed;
                        } else {
                            output = threshold(POutput, Thresh, -Thresh);
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
            }
            if (error <= range && error >= -range && aiming) {
                ((DcMotorEx)robot.FL).setVelocity(0);
                ((DcMotorEx)robot.FR).setVelocity(0);
                ((DcMotorEx)robot.BL).setVelocity(0);
                ((DcMotorEx)robot.BR).setVelocity(0);
                aiming = false;
                firstTime = true;
                telemetry.addData("Say", "Aimed");
                sleep(100);
            }
            if (gamepad1.a) {
                if (Target != null) {
                    aiming = true;
                    loopTime.reset();
                }
            }
            telemetry.update();
        }
        telemetry.update();
    }

    //create pipeline
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
            byte[] lookUpTableData = new byte[(int) (lookUpTable.total()*lookUpTable.channels())];
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
        private byte saturate(double val)
        {
            int iVal = (int) Math.round(val);
            iVal = iVal > 255 ? 255 : (iVal < 0 ? 0 : iVal);
            return (byte) iVal;
        }

    }
    static double threshold(double value, double max, double min)
    {
        if (value > max)
        {
            value = max;
        }
        else if(value < min)
        {
            value = min;
        }
        else
        {
            value = value;
        }
        return value;
    }
    public void drop()
    {
        robot.wobbleMotor.setPower(0.5);
        while(wobblePos<= -50)
        {
           wobblePos = ((DcMotorEx)robot.wobbleMotor).getCurrentPosition();
        }
        robot.wobbleMotor.setPower(0);
        robot.wobbleServo.setPosition(0);
        if(isHoldingWobble)
        {
            robot.wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.wobbleMotor.setPower(0);
        }
    }
    public void raise()
    {
        if(isHoldingWobble == false)
        {
            robot.wobbleServo.setPosition(0.25);
            sleep(200);
            isHoldingWobble = true;
        }
        else
        {
            isHoldingWobble = false;
        }
        robot.wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.wobbleMotor.setPower(-0.5);
        while(wobblePos>=-550)
        {
            wobblePos = ((DcMotorEx)robot.wobbleMotor).getCurrentPosition();
        }
        robot.wobbleMotor.setPower(0);
    }
}

