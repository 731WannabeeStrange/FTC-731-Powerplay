package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.autonomous.roadrunner.drive.T265Localizer;
import org.firstinspires.ftc.teamcode.utils.GridGuider;

@TeleOp(group="test")
public class GridGuideTestOpMode extends LinearOpMode {
    FtcDashboard dashboard;
    DcMotorEx fl, fr, rl, rr;
    public double FL_power, FR_power, RL_power, RR_power;

    public static double P = 0.04;
    public double desiredAngle = 0;
    double previousError, error;
    public String turnState = "auto";

    public double denominator;

    public enum DriveMode {
        DRIVER_CONTROLLED,
        AUTO_CONTROL
    }
    public DriveMode driveState = DriveMode.AUTO_CONTROL;
    
    T265Localizer localizer;
    GridGuider gridGuider;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        localizer = new T265Localizer(hardwareMap, true);
        
        gridGuider = new GridGuider(15.5, 15.5);

        fl = hardwareMap.get(DcMotorEx.class, "flMotor");
        fr = hardwareMap.get(DcMotorEx.class, "frMotor");
        rl = hardwareMap.get(DcMotorEx.class, "rlMotor");
        rr = hardwareMap.get(DcMotorEx.class, "rrMotor");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            localizer.update();

            Vector2d repulsed = gridGuider.calculateRepulsedVector(localizer.getPoseEstimate());
            
            double y = gamepad1.left_stick_y + repulsed.getY();
            double x = gamepad1.left_stick_x + repulsed.getX();
            double rx = gamepad1.right_stick_x;

            double gyro_radians = localizer.getHeading() * Math.PI/180;
            double newForward = y * Math.cos(gyro_radians) + x * Math.sin(gyro_radians);
            double newStrafe = -y * Math.sin(gyro_radians) + x * Math.cos(gyro_radians);

            error = angleWrap(desiredAngle - localizer.getHeading());

            telemetry.addData("Turn Error", error);

            double rcw = P * -error;

            previousError = error;

            telemetry.addData("Read angle", localizer.getHeading());
            telemetry.addData("Desired Angle", desiredAngle);
            telemetry.addData("rightStickX", rx);

            switch (driveState) {
                case AUTO_CONTROL:
                    if (gamepad1.dpad_up) {
                        desiredAngle = 0;
                    }
                    if (gamepad1.dpad_left) {
                        desiredAngle = 270;
                    }
                    if (gamepad1.dpad_down) {
                        desiredAngle = 180;
                    }
                    if (gamepad1.dpad_right) {
                        desiredAngle = 90;
                    }

                    if (rx != 0) {
                        driveState = DriveMode.DRIVER_CONTROLLED;
                    }
                    turnState = "auto";
                    //denominator = Math.max(Math.abs(newForward) + Math.abs(newStrafe) + Math.abs(rcw), 1);
                    FL_power = (-newForward + newStrafe + rcw);// / denominator;
                    RL_power = (-newForward - newStrafe + rcw);// / denominator;
                    FR_power = (-newForward - newStrafe - rcw);// / denominator;
                    RR_power = (-newForward + newStrafe - rcw);// / denominator;
                    break;
                case DRIVER_CONTROLLED:
                    turnState = "driver";
                    denominator = Math.max(Math.abs(newForward) + Math.abs(newStrafe) + Math.abs(rx), 1);
                    FL_power = (-newForward + newStrafe + rx) / denominator;
                    RL_power = (-newForward - newStrafe + rx) / denominator;
                    FR_power = (-newForward - newStrafe - rx) / denominator;
                    RR_power = (-newForward + newStrafe - rx) / denominator;
                    if (rx == 0) {
                        desiredAngle = localizer.getHeading();
                        driveState = DriveMode.AUTO_CONTROL;
                    }
                    if (gamepad1.dpad_up) {
                        desiredAngle = 0;
                        driveState = DriveMode.AUTO_CONTROL;
                    }
                    if (gamepad1.dpad_left) {
                        desiredAngle = 270;
                        driveState = DriveMode.AUTO_CONTROL;
                    }
                    if (gamepad1.dpad_down) {
                        desiredAngle = 180;
                        driveState = DriveMode.AUTO_CONTROL;
                    }
                    if (gamepad1.dpad_right) {
                        desiredAngle = 90;
                        driveState = DriveMode.AUTO_CONTROL;
                    }
                    break;
            }

            telemetry.addData("turnState", turnState);

            if (gamepad1.left_bumper) {
                FL_power /= 4;
                FR_power /= 4;
                RL_power /= 4;
                RR_power /= 4;
            }

            telemetry.addData("RCW", rcw);
            telemetry.addData("FLpower", FL_power);
            telemetry.addData("FRpower", FR_power);
            telemetry.addData("RLpower", RL_power);
            telemetry.addData("RRpower", RR_power);

            fl.setPower(FL_power);
            fr.setPower(FR_power);
            rl.setPower(RL_power);
            rr.setPower(RR_power);
        }
    }

    public double angleWrap(double angle) {
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }

        return angle;
    }
}
