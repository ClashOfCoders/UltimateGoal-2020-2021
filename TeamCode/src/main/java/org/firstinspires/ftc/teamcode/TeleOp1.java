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
@TeleOp(name= "TeleOp V.1")
public class TeleOp1 extends LinearOpMode {
    //create webcam and variables for target detection
    OpenCvCamera webcam;
    static RotatedRect rotatedRectFitToContour;
    static RotatedRect Target;
    boolean aiming = false;
    double wobblePos = 0;
    boolean wobbleUp = false;
    boolean powerShot = false;
    double FrontLeft, FrontRight,RearLeft, RearRight, r, robotAngle, rightX;
    boolean shoot = false;
    double error = 1000;
    double output;
    double P = 2;
    double POutput;
    double minSpeed = 0;
    double endPoint = 470;
    double powerShot1 = 185;
    double powerShot2 = 267;
    double powerShot3 = 350;
    double targetPower1 = 1660;
    double targetPower2 = 1660;
    double targetPower3 = 16609;
    double startPoint;
    double range = 5;
    double shooterSpeed = 1000;
    double a = -0.00278928;
    double b = 1.05409;
    double c = -126.017;
    double d = 6595.33;
    double powerMultiplier = .5;
    int topSpeed = 4000;
    double D = 1.2;
    double DOutput;
    boolean firstTime = true;
    double previousError = 1000;
    double Thresh = 300;
    double motorSpeed;
    boolean armMovement;
    boolean wol = true;
    double distance;
    ElapsedTime loopTime = new ElapsedTime();


    //create odometry coordinate object
    /* Declare OpMode members. */
    Hardware2021 robot = new Hardware2021();   // Use hardware map

    @Override
    public void runOpMode() throws InterruptedException {

        //drive variables



        //Start opencv camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "shooterCam"), cameraMonitorViewId);

        webcam.setPipeline(new TargetDetectionPipeline());

        //start camera stream
        webcam.openCameraDevice();
        webcam.startStreaming(864, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);


        //set framerate
        telemetry.setMsTransmissionInterval(20);

        //init hardware and set servo pos
        robot.init(hardwareMap);
        robot.pusher.setPosition(0.666);
        robot.wobbleServo.setPosition(0.4013);
        ((DcMotorEx)robot.FR).setVelocityPIDFCoefficients(30,0,0,0);
        ((DcMotorEx)robot.FL).setVelocityPIDFCoefficients(30,0,0,0);
        ((DcMotorEx)robot.BR).setVelocityPIDFCoefficients(30,0,0,0);
        ((DcMotorEx)robot.BL).setVelocityPIDFCoefficients(30,0,0,0);
        ((DcMotorEx)robot.shooter).setVelocityPIDFCoefficients(900,1.2,0,0);
        armMovement();
        //send telemetry messge to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");

        telemetry.update();

        //wait for the game to start (driver presses PLAY)
        waitForStart();
        //globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, 305.5774907364, 75);
        //Thread positionThread = new Thread(globalPositionUpdate);
        //positionThread.start();
        //globalPositionUpdate.reverseNormalEncoder();

        //run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double motorSpeed = ((DcMotorEx) robot.shooter).getVelocity();
            if (gamepad2.x) {
                armMovement = true;
            }
            if(gamepad2.left_trigger > 0.5)
            {
                wol = true;
            }
            else
            {
                wol = false;
            }
            if (gamepad2.dpad_up)
            {
                shooterSpeed += 10;
                sleep(100);
            }
            if (gamepad2.dpad_down)
            {
                shooterSpeed -= 10;
                sleep(100);
            }
            if(gamepad2.dpad_right)
            {
                ((DcMotorEx) robot.shooter).setVelocity(shooterSpeed);
            }
            if(gamepad2.dpad_left)
            {
                ((DcMotorEx) robot.shooter).setVelocity(0);
            }

            if(armMovement)
            {
                if(wobbleUp) {
                    robot.wobbleMotor.setPower(.5);
                    wobblePos = ((DcMotorEx) robot.wobbleMotor).getCurrentPosition();
                    if(wol)
                    {
                        while (wobblePos <= 335) {
                            r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                            robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                            if(gamepad1.right_trigger>=0.85) {
                                rightX = -gamepad1.right_stick_x * .725;
                            }
                            else
                            {
                                rightX = -gamepad1.right_stick_x*0.65;
                            }
                            FrontLeft = r * Math.cos(robotAngle) - rightX;
                            FrontRight = r * Math.sin(robotAngle) + rightX;
                            RearLeft = r * Math.sin(robotAngle) - rightX;
                            RearRight = r * Math.cos(robotAngle) + rightX;
                            //set values to motor powers
                            ((DcMotorEx) robot.FL).setVelocity(FrontLeft*topSpeed*powerMultiplier);
                            ((DcMotorEx) robot.FR).setVelocity(FrontRight*topSpeed*powerMultiplier);
                            ((DcMotorEx) robot.BR).setVelocity(RearRight*topSpeed*powerMultiplier);
                            ((DcMotorEx) robot.BL).setVelocity(RearLeft*topSpeed*powerMultiplier);
                            wobblePos = ((DcMotorEx) robot.wobbleMotor).getCurrentPosition();
                        }
                        robot.wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        robot.wobbleMotor.setPower(0);
                        robot.wobbleServo.setPosition(0);
                        wobbleUp = false;
                        armMovement = false;
                    }
                    else
                    {
                        while (wobblePos <= 640) {
                            r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                            robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                            if(gamepad1.right_trigger>=0.85) {
                                rightX = -gamepad1.right_stick_x * .725;
                            }
                            else
                            {
                                rightX = -gamepad1.right_stick_x*0.65;
                            }
                            FrontLeft = r * Math.cos(robotAngle) - rightX;
                            FrontRight = r * Math.sin(robotAngle) + rightX;
                            RearLeft = r * Math.sin(robotAngle) - rightX;
                            RearRight = r * Math.cos(robotAngle) + rightX;
                            //set values to motor powers
                            ((DcMotorEx) robot.FL).setVelocity(FrontLeft*topSpeed*powerMultiplier);
                            ((DcMotorEx) robot.FR).setVelocity(FrontRight*topSpeed*powerMultiplier);
                            ((DcMotorEx) robot.BR).setVelocity(RearRight*topSpeed*powerMultiplier);
                            ((DcMotorEx) robot.BL).setVelocity(RearLeft*topSpeed*powerMultiplier);
                            wobblePos = ((DcMotorEx) robot.wobbleMotor).getCurrentPosition();
                        }
                        robot.wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    }
                    robot.wobbleMotor.setPower(0);
                    robot.wobbleServo.setPosition(0);
                    wobbleUp = false;

                }
                else{
                    robot.wobbleServo.setPosition(.4065);
                    sleep(700);
                    robot.wobbleMotor.setPower(-.5);
                    sleep(100);
                    wobblePos = ((DcMotorEx)robot.wobbleMotor).getVelocity();
                    while (wobblePos<= -30) {
                        r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                        robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                        if(gamepad1.right_trigger>=0.85) {
                            rightX = -gamepad1.right_stick_x * .725;
                        }
                        else
                        {
                            rightX = -gamepad1.right_stick_x*0.65;
                        }
                        FrontLeft = r * Math.cos(robotAngle) - rightX;
                        FrontRight = r * Math.sin(robotAngle) + rightX;
                        RearLeft = r * Math.sin(robotAngle) - rightX;
                        RearRight = r * Math.cos(robotAngle) + rightX;
                        //set values to motor powers
                        ((DcMotorEx) robot.FL).setVelocity(FrontLeft*topSpeed*powerMultiplier);
                        ((DcMotorEx) robot.FR).setVelocity(FrontRight*topSpeed*powerMultiplier);
                        ((DcMotorEx) robot.BR).setVelocity(RearRight*topSpeed*powerMultiplier);
                        ((DcMotorEx) robot.BL).setVelocity(RearLeft*topSpeed*powerMultiplier);
                        wobblePos = ((DcMotorEx) robot.wobbleMotor).getVelocity();
                    }
                    robot.wobbleMotor.setPower(0);
                    sleep(100);
                    robot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    wobbleUp = true;
                }
                armMovement = false;
            }
            if (Target != null) {
                telemetry.addData("Motor Speed", motorSpeed);
                telemetry.addData("Set Speed", shooterSpeed);
                telemetry.addData("Width", Target.size.width);
                telemetry.addData("Height", Target.size.height);
                telemetry.addData("Center y", Target.center.y);
                telemetry.update();
            }
            telemetry.addData("wobbleServoPosition", robot.wobbleServo.getPosition());
            telemetry.addData("wobbleMotorPos", ((DcMotorEx) robot.wobbleMotor).getCurrentPosition());
            telemetry.addData("FL Speed", ((DcMotorEx)robot.FL).getCurrentPosition());
            telemetry.addData("FR Speed", ((DcMotorEx)robot.FR).getCurrentPosition());
            telemetry.addData("BL Speed", ((DcMotorEx)robot.BL).getCurrentPosition());
            telemetry.addData("BR Speed", ((DcMotorEx)robot.BR).getCurrentPosition());
            telemetry.update();


            //do calculations to convert stick position to motor power
            if(aiming == false) {
                r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                if(gamepad1.right_trigger>=0.85) {
                    rightX = -gamepad1.right_stick_x * 0.725;
                }
                else
                {
                    rightX = -gamepad1.right_stick_x * 0.65;
                }
                FrontLeft = r * Math.cos(robotAngle) - rightX;
                FrontRight = r * Math.sin(robotAngle) + rightX;
                RearLeft = r * Math.sin(robotAngle) - rightX;
                RearRight = r * Math.cos(robotAngle) + rightX;
                //set values to motor powers
                ((DcMotorEx) robot.FL).setVelocity(FrontLeft*topSpeed*powerMultiplier);
                ((DcMotorEx) robot.FR).setVelocity(FrontRight*topSpeed*powerMultiplier);
                ((DcMotorEx) robot.BR).setVelocity(RearRight*topSpeed*powerMultiplier);
                ((DcMotorEx) robot.BL).setVelocity(RearLeft*topSpeed*powerMultiplier);

            }

            if (aiming) {
                if(firstTime) {
                    ((DcMotorEx)robot.FL).setVelocityPIDFCoefficients(30,10,2,0);
                    ((DcMotorEx)robot.FR).setVelocityPIDFCoefficients(30,10,2,0);
                    ((DcMotorEx)robot.BL).setVelocityPIDFCoefficients(30,10,2,0);
                    ((DcMotorEx)robot.BR).setVelocityPIDFCoefficients(30,10,2,0);
                    startPoint = Target.center.y;
                    previousError = endPoint - startPoint;
                    firstTime = false;
                }
                sleep(75-((int)loopTime.milliseconds()));
                startPoint = Target.center.y;
                if (Target.size.height < Target.size.width) {
                    distance = Target.size.height;
                } else {
                    distance = Target.size.width;
                }
                double targetSpeed = (a * (distance * distance * distance)) + (b * (distance * distance)) + (c * distance) + d;
                ((DcMotorEx) robot.shooter).setVelocity(targetSpeed);
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
                sleep(300);
                if (error >= range || error <= -range) {
                    aiming = true;
                } else {
                    if (Target.size.height < Target.size.width) {
                        distance = Target.size.height;
                    } else {
                        distance = Target.size.width;
                    }
                    double targetSpeed = (a * (distance * distance * distance)) + (b * (distance * distance)) + (c * distance) + d;
                    ((DcMotorEx) robot.shooter).setVelocity(targetSpeed);
                    while (motorSpeed < targetSpeed * 0.989 && opModeIsActive() || motorSpeed >= (targetSpeed * 1.011) && opModeIsActive()) {
                        motorSpeed = ((DcMotorEx) robot.shooter).getVelocity();
                        telemetry.addData("Set point", targetSpeed);
                        telemetry.addData("Motor Speed", motorSpeed);
                        telemetry.update();
                    }
                    aiming = false;
                    firstTime = true;
                    for (int i = 0; i <= 2; i++) {
                        motorSpeed = ((DcMotorEx) robot.shooter).getVelocity();
                        while (motorSpeed < targetSpeed * 0.989 && opModeIsActive() || motorSpeed >= (targetSpeed * 1.011) && opModeIsActive()) {
                            motorSpeed = ((DcMotorEx) robot.shooter).getVelocity();
                            telemetry.addData("Say", "Shooting");
                            telemetry.addData("Set point", targetSpeed);
                            telemetry.addData("Motor Speed", motorSpeed);
                            telemetry.update();
                        }
                        ((DcMotorEx)robot.FR).setVelocityPIDFCoefficients(30,0,0,0);
                        ((DcMotorEx)robot.FL).setVelocityPIDFCoefficients(30,0,0,0);
                        ((DcMotorEx)robot.BR).setVelocityPIDFCoefficients(30,0,0,0);
                        ((DcMotorEx)robot.BL).setVelocityPIDFCoefficients(30,0,0,0);
                        robot.pusher.setPosition(1);
                        sleep(300);
                        robot.pusher.setPosition(0.666);
                        sleep(300);
                    }
                    ((DcMotorEx) robot.shooter).setVelocity(0);
                }

            }
            if(gamepad2.y)
            {
                powerShot = true;
                loopTime.reset();
                while(powerShot && opModeIsActive())
                {
                    powerShot = powerShots(Target, powerShot1, targetPower1, 5);
                    loopTime.reset();
                }
                powerShot = true;
                loopTime.reset();
                while(powerShot && opModeIsActive())
                {
                    powerShot = powerShots(Target, powerShot2, targetPower2, 5);
                    loopTime.reset();
                }
                powerShot = true;
                loopTime.reset();
                while(powerShot && opModeIsActive())
                {
                    powerShot = powerShots(Target, powerShot3, targetPower3, 5);
                    loopTime.reset();
                }
                ((DcMotorEx) robot.FR).setVelocityPIDFCoefficients(30, 0, 0, 0);
                ((DcMotorEx) robot.FL).setVelocityPIDFCoefficients(30, 0, 0, 0);
                ((DcMotorEx) robot.BR).setVelocityPIDFCoefficients(30, 0, 0, 0);
                ((DcMotorEx) robot.BL).setVelocityPIDFCoefficients(30, 0, 0, 0);
                robot.shooter.setPower(0);
            }

            if (gamepad2.right_trigger >= 0.65) {
                ((DcMotorEx) robot.shooter).setVelocity(1700);
            }
            if(gamepad1.right_trigger>=.85){
                powerMultiplier = 1;
            }
            else {
                powerMultiplier = 0.35;
            }
            if (gamepad2.right_bumper) {
                robot.intake.setPower(1.00);
            } else if (gamepad2.left_bumper) {
                robot.intake.setPower(-1.00);
            } else {
                robot.intake.setPower(0);
            }

            if (gamepad2.a) {
                if (Target != null) {
                    aiming = true;
                    loopTime.reset();
                }
            }
            if(gamepad2.b) {
                aiming = false;
            }


            if (Math.abs(gamepad1.left_stick_x) >= 0.25 && shoot || Math.abs(gamepad1.left_stick_y) >= 0.25 && shoot || Math.abs(gamepad1.right_stick_x) >= 0.25 && shoot) {
                ((DcMotorEx) robot.shooter).setVelocity(0);
                shoot = false;
            }
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
    public void armMovement()
    {
        int target=0;
        if(wobbleUp) {
            robot.wobbleMotor.setPower(.5);
            wobblePos = ((DcMotorEx) robot.wobbleMotor).getCurrentPosition();

            if(wol)
            {
                while (wobblePos <= 335) {
                    wobblePos = ((DcMotorEx) robot.wobbleMotor).getCurrentPosition();
                }
                robot.wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            else
            {
                while (wobblePos <= 680) {
                    wobblePos = ((DcMotorEx) robot.wobbleMotor).getCurrentPosition();
                }
                robot.wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            robot.wobbleMotor.setPower(0);
            robot.wobbleServo.setPosition(0);
            wobbleUp = false;

        }
        else{
            robot.wobbleServo.setPosition(.4065);
            sleep(700);
            robot.wobbleMotor.setPower(-.5);
            sleep(100);
            wobblePos = ((DcMotorEx)robot.wobbleMotor).getVelocity();
            while (wobblePos<= -30) {
                wobblePos = ((DcMotorEx) robot.wobbleMotor).getVelocity();
            }
            robot.wobbleMotor.setPower(0);
            sleep(100);
            robot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wobbleUp = true;
        }
    }
    public boolean powerShots(RotatedRect aimAt, double endPoint, double shootSpeed, double range)
    {
        if(firstTime) {
            ((DcMotorEx)robot.FL).setVelocityPIDFCoefficients(30,10,2,0);
            ((DcMotorEx)robot.FR).setVelocityPIDFCoefficients(30,10,2,0);
            ((DcMotorEx)robot.BL).setVelocityPIDFCoefficients(30,10,2,0);
            ((DcMotorEx)robot.BR).setVelocityPIDFCoefficients(30,10,2,0);
            startPoint = aimAt.center.y;
            previousError = endPoint - startPoint;
            firstTime = false;
            ((DcMotorEx) robot.shooter).setVelocity(shootSpeed);
            loopTime.reset();
        }

        sleep(75-((int)loopTime.milliseconds()));
        startPoint = aimAt.center.y;
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
            telemetry.update();
            return true;
        }
        if (error <= range && error >= -range) {
            robot.FL.setPower(0);
            robot.FR.setPower(0);
            robot.BL.setPower(0);
            robot.BR.setPower(0);

            sleep(200);
            error = endPoint - startPoint;
            if (error >= range || error <= -range)
            {
                loopTime.reset();
                return true;
            }
            else
            {
                ((DcMotorEx) robot.shooter).setVelocity(shootSpeed);
                motorSpeed = ((DcMotorEx) robot.shooter).getVelocity();
                while (motorSpeed < shootSpeed * 0.989 && opModeIsActive() || motorSpeed >= (shootSpeed * 1.011) && opModeIsActive()) {
                    ((DcMotorEx) robot.shooter).setVelocity(shootSpeed);
                    motorSpeed = ((DcMotorEx) robot.shooter).getVelocity();
                    telemetry.addData("Set point", shootSpeed);
                    telemetry.addData("Motor Speed", motorSpeed);
                    telemetry.update();
                }
                sleep(50);
                motorSpeed = ((DcMotorEx) robot.shooter).getVelocity();
                while (motorSpeed < shootSpeed * 0.989 && opModeIsActive() || motorSpeed >= (shootSpeed * 1.011) && opModeIsActive()) {
                    ((DcMotorEx) robot.shooter).setVelocity(shootSpeed);
                    motorSpeed = ((DcMotorEx) robot.shooter).getVelocity();
                    telemetry.addData("Say", "Shooting");
                    telemetry.addData("Set point", shootSpeed);
                    telemetry.addData("Motor Speed", motorSpeed);
                    telemetry.update();
                }
                robot.pusher.setPosition(1);
                sleep(300);
                robot.pusher.setPosition(0.666);
                sleep(300);
                firstTime = true;
                return false;
            }
        }
        return true;
    }

/*    public void raise()
    {
        if(isHoldingWobble == false)
        {
            robot.wobbleServo.setPosition(0.3765);
            sleep(400);
            isHoldingWobble = true;
        }
        else
        {
            isHoldingWobble = false;
        }
        robot.wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.wobbleMotor.setPower(-0.5);
        sleep(200);
        wobblePos = ((DcMotorEx)robot.wobbleMotor).getVelocity();
        while(wobblePos <= -100)
        {
            wobblePos = ((DcMotorEx)robot.wobbleMotor).getVelocity();
        }
        robot.wobbleMotor.setPower(0);
        robot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }*/
}

