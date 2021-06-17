package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware2021;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    double speed;
    double speed2;
    double avgSpeed;
    double targetSpeed = 1650;
    double speedAtShoot;
    double servoPush = 0.164;
    double p = 150;
    double i = 0;
    double d = 0;
    double f = 13.5;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        final Hardware2021 robot = new Hardware2021();
        robot.init(hardwareMap);

        robot.shooter.setVelocityPIDFCoefficients(p,i,d,f);
        robot.shooter.setVelocityPIDFCoefficients(p, i, d, f);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            if(gamepad2.right_bumper) {
                robot.shooter.setVelocity(targetSpeed);
                robot.shooter2.setVelocity(targetSpeed);
            }

            if(gamepad2.left_bumper) {
                robot.shooter.setVelocity(0);
                robot.shooter2.setVelocity(0);
            }

            if(gamepad1.dpad_up){
                p += 10;
                robot.shooter.setVelocityPIDFCoefficients(p, i, d, f);
                sleep(200);
            }

            if(gamepad1.dpad_down){
                p -= 10;
                robot.shooter.setVelocityPIDFCoefficients(p, i, d, f);
                sleep(200);
            }

            if(gamepad1.dpad_right) {
                i += 1;
                robot.shooter.setVelocityPIDFCoefficients(p, i, d, f);
                sleep(200);
            }

            if(gamepad1.dpad_left) {
                i -= 1;
                robot.shooter.setVelocityPIDFCoefficients(p, i, d, f);
                sleep(200);
            }

            if(gamepad1.y) {
                d += 1;
                robot.shooter.setVelocityPIDFCoefficients(p, i, d, f);
                sleep(200);
            }

            if(gamepad1.a) {
                d -= 1;
                robot.shooter.setVelocityPIDFCoefficients(p, i, d, f);
                sleep(200);
            }

            if(gamepad1.b){
                f += 1;
                robot.shooter.setVelocityPIDFCoefficients(p, i, d, f);
                sleep(200);
            }

            if(gamepad1.x){
                f-= 1;
                robot.shooter.setVelocityPIDFCoefficients(p, i, d, f);
                sleep(200);
            }

            if(gamepad1.right_trigger > 0.5) {
                targetSpeed += 100;
                sleep(200);
            }

            if(gamepad1.left_trigger > 0.5) {
                targetSpeed -= 100;
                sleep(200);
            }

            if(gamepad2.dpad_up)
            {
                servoPush += 0.001;
                sleep(200);
            }

            if(gamepad2.dpad_down)
            {
                servoPush -= 0.001;
                sleep(200);
            }

            if(gamepad2.a)
            {
                robot.pusher.setPosition(servoPush);
                timer.reset();
                speedAtShoot = robot.shooter.getVelocity();
                while(timer.milliseconds() < 115)
                {
                    speed = robot.shooter.getVelocity();
                    speed2 = robot.shooter2.getVelocity();
                    avgSpeed = (speed+speed2)/2;
                    telemetry.addData("Motor Speed", avgSpeed);
                    telemetry.addData("Target Speed", targetSpeed);
                    telemetry.update();
                }
                robot.pusher.setPosition(0.08);
                timer.reset();
                while(timer.milliseconds() < 115)
                {
                    speed = robot.shooter.getVelocity();
                    speed2 = robot.shooter2.getVelocity();
                    avgSpeed = (speed+speed2)/2;
                    telemetry.addData("Motor Speed", avgSpeed);
                    telemetry.addData("Target Speed", targetSpeed);
                    telemetry.update();
                }
            }

            speed = robot.shooter.getVelocity();
            speed2 = robot.shooter2.getVelocity();
            avgSpeed = (speed+speed2)/2;
            telemetry.addData("Motor Speed", avgSpeed);
            telemetry.addData("Target Speed", targetSpeed);

            /*telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);
            telemetry.addData("F", f);
            telemetry.addData("Servo Val", servoPush);
            telemetry.addData("Motor Speed At Shot", speedAtShoot);
            telemetry.addData("Target Speed", targetSpeed);
             */
            telemetry.update();
        }
    }
}
