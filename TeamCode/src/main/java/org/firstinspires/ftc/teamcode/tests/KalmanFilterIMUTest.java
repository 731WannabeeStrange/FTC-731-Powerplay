package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.firstinspires.ftc.teamcode.utils.KalmanFilter;

import java.util.Arrays;

@TeleOp(group="test")
public class KalmanFilterIMUTest extends LinearOpMode {
    BNO055IMU imu;
    Drivetrain dt;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new Drivetrain(hardwareMap, telemetry);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        KalmanFilter filter = new KalmanFilter(0.5, 0.5, 5);

        waitForStart();

        while (opModeIsActive()) {
            dt.driveRobot(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    false,
                    false,
                    false,
                    false,
                    false
            );

            double imuReading = imu.getAngularOrientation().firstAngle;
            Dashboard.packet.put("IMU Reading", imuReading);
            Dashboard.packet.put("Kalman Reading", filter.estimate(imuReading));
            Dashboard.packet.put("Kalman X", filter.getX());

            Dashboard.packet.addLine("");
            Dashboard.packet.addLine(Arrays.toString(filter.estimates.toArray()));

            Dashboard.periodic();
        }
    }
}
