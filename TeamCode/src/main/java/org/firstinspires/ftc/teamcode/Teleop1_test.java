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


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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

import java.lang.reflect.Array;
import java.util.ArrayList;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.*;

@TeleOp(name= "TeleOp V.1 Test")
public class Teleop1_test extends LinearOpMode {

    final Hardware2021 robot = new Hardware2021();   // Use hardware map
    double servoOut = 0.08;
    double servoIn = 0.164;
    long servoSleep = 115;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        robot.pusher.setPosition(servoOut);
        robot.shooter.setVelocityPIDFCoefficients(400, 0, 0.3, 14);
        robot.shooter2.setVelocityPIDFCoefficients(400, 0, 0.3, 14);

        waitForStart();

        while(opModeIsActive() && !isStopRequested())
        {
            if(gamepad1.dpad_up)
            {
                servoIn += 0.001;
                sleep(100);
            }

            if(gamepad1.dpad_down)
            {
                servoIn -= 0.001;
                sleep(100);
            }

            if(gamepad1.dpad_right)
            {
                servoOut += 0.001;
                sleep(100);
            }

            if(gamepad1.dpad_left)
            {
                servoOut -= 0.001;
                sleep(100);
            }

            if(gamepad1.y)
            {
                robot.pusher.setPosition(servoOut);
            }

            if(gamepad1.a)
            {
                robot.pusher.setPosition(servoIn);
            }

            if(gamepad1.b)
            {
                servoSleep += 1;
                sleep(100);
            }

            if(gamepad1.x)
            {
                servoSleep -= 1;
                sleep(100);
            }

            if(gamepad1.right_bumper)
            {
                robot.shooter.setVelocity(1600);
            }

            if(gamepad1.left_trigger > 0.5)
            {
                robot.shooter2.setVelocity(1600);
            }

            if(gamepad1.left_bumper)
            {
                robot.shooter.setVelocity(0);
                robot.shooter2.setVelocity(0);
            }

            if(gamepad1.right_trigger > 0.5)
            {
                robot.pusher.setPosition(servoIn);
                sleep(servoSleep);
                robot.pusher.setPosition(servoOut);
                sleep(servoSleep);
            }

            telemetry.addData("Motor 1 Velo", robot.shooter.getVelocity());
            telemetry.addData("Motor 2 Velo", robot.shooter2.getVelocity());
            telemetry.update();

        }




    }
}

