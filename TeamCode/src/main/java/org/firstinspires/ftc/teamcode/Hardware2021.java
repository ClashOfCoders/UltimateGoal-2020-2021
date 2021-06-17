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
 * "AS IS" AND ANY EXPRESS OR IMPLIED WABRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WABRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTEBRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
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
 * "AS IS" AND ANY EXPRESS OR IMPLIED WABRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WABRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTEBRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware2021
{

    //Odometry Wheels

    /* Public OpMode members. */
    public DcMotor  FR  = null;
    public DcMotor  FL  = null;
    public DcMotor  BR  = null;
    public DcMotor  BL  = null;
    public DcMotorEx shooter = null;
    public DcMotorEx shooter2 = null;
    public DcMotor intake = null;
    public Servo pusher = null;
    public DcMotorEx wobbleMotor = null;
    public Servo wobbleServo = null;
    public DistanceSensor ringSensor = null;
    public RevBlinkinLedDriver ledStrip = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware2021(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        FR  = hwMap.get(DcMotor.class, "FR");
        FL  = hwMap.get(DcMotor.class, "FL");
        BR  = hwMap.get(DcMotor.class, "BR");
        BL  = hwMap.get(DcMotor.class, "BL");
        shooter = hwMap.get(DcMotorEx.class, "Shooter");
        shooter2 = hwMap.get(DcMotorEx.class, "Shooter2");
        intake = hwMap.get(DcMotor.class, "Intake");
        wobbleMotor = hwMap.get(DcMotorEx.class, "WobbleMotor");

        ledStrip = hwMap.get(RevBlinkinLedDriver.class, "LEDStrip");




        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        wobbleMotor.setDirection(DcMotor.Direction.FORWARD);



        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // Set all motors to run without encoders.
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        pusher = hwMap.get(Servo.class, "Pusher");
        wobbleServo = hwMap.get(Servo.class, "WobbleServo");


        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //sensors
        ringSensor = hwMap.get(DistanceSensor.class, "RingSensor");

        ledStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);


    }
 }