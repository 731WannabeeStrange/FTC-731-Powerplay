package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.firstinspires.ftc.teamcode.utils.KalmanFilter;

import java.util.Arrays;
import java.util.Random;

@Config
@TeleOp
public class KalmanFilterTest extends LinearOpMode {
    public static double Q = 0.5;
    public static double R = 2;

    BNO055IMU imu;
    Drivetrain dt;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new Drivetrain(hardwareMap, telemetry);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        KalmanFilter filter = new KalmanFilter(Q, R, 5);

        double sensorReading = 1000;

        waitForStart();

        while (opModeIsActive()) {

            Random random = new Random();
            sensorReading = (random.nextDouble() - 0.5) * 0.1;
            Dashboard.packet.put("Sensor Reading", sensorReading);
            Dashboard.packet.put("Kalman Reading", filter.estimate(sensorReading));

            Dashboard.packet.addLine("");
            Dashboard.packet.addLine(Arrays.toString(filter.estimates.toArray()));

            Dashboard.periodic();

            sleep(200);
        }
    }
}
