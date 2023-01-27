package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

@TeleOp(group="test")
@Config
public class T265TestOpMode extends LinearOpMode
{
    // This is the transformation between the center of the camera and the center of the robot
    Transform2d cameraToRobot = new Transform2d();
    // Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
    double encoderMeasurementCovariance = 0.8;
    // Set to the starting pose of the robot
    Pose2d startingPose = new Pose2d(0, 0, new Rotation2d());

    //private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        T265Camera slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
        slamra.setPose(startingPose); // Useful if your robot doesn't start at the field-relative origin

        // Call this when you're ready to get camera updates
        slamra.start();

        waitForStart();
        while (opModeIsActive()) {
            T265Camera.CameraUpdate latestUpdate = slamra.getLastReceivedCameraUpdate();
            Pose2d latestPose = latestUpdate.pose;
            telemetry.addData("x", latestPose.getX());
            telemetry.addData("y", latestPose.getY());
            telemetry.addData("heading", Math.toDegrees(latestPose.getHeading()));
            telemetry.addData("confidence", latestUpdate.confidence);
            telemetry.update();
        }
    }
}