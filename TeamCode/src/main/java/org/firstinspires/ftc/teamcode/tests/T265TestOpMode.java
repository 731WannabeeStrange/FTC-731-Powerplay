package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;
import com.spartronics4915.lib.T265Helper;


@TeleOp
@Config
public class T265TestOpMode extends LinearOpMode
{
    T265Camera slamera;

    //private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        slamera = T265Helper.getCamera(
                new T265Camera.OdometryInfo(
                        new Pose2d(), 0.8
                ), hardwareMap.appContext);

        System.out.println("731: camera initialized");
        // This is the transformation between the center of the camera and the center of the robot
        Pose2d cameraToRobot = new Pose2d();
        // Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
        double encoderMeasurementCovariance = 0.8;
        // Set to the starting pose of the robot
        Pose2d startingPose = new Pose2d();

        slamera.setOdometryInfo(cameraToRobot, encoderMeasurementCovariance);

        slamera.setPose(startingPose); // Useful if your robot doesn't start at the field-relative origin

        while (opModeIsActive()) {
            telemetry.addData("x", slamera.getLastReceivedCameraUpdate().pose.getX());
            telemetry.addData("y", slamera.getLastReceivedCameraUpdate().pose.getY());
            telemetry.addData("heading", slamera.getLastReceivedCameraUpdate().pose.getHeading());

            telemetry.update();
        }
    }
}