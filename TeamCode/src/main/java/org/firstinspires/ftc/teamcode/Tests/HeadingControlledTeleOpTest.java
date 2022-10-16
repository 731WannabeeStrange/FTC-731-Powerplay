package org.firstinspires.ftc.teamcode.Tests;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
@TeleOp
public class HeadingControlledTeleOpTest extends LinearOpMode {
    public static double turnSpeed = 0.5;
    public static double P = 1;
    public static double I = 0;
    public static double D = 0;

    DcMotorEx fl, fr, bl, br;
    BNO055IMU imu;
    double desiredAngle = 0;
    double previousError;
    double integral;
    ElapsedTime eTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotorEx.class, "flMotor");
        fr = hardwareMap.get(DcMotorEx.class, "frMotor");
        bl = hardwareMap.get(DcMotorEx.class, "rlMotor");
        br = hardwareMap.get(DcMotorEx.class, "rrMotor");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample op mode
        parameters.mode = BNO055IMU.SensorMode.IMU;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(null, null, 1000);


        waitForStart();

        eTime.reset();

        while (opModeIsActive() && !isStopRequested()) {
            time = eTime.time();
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            desiredAngle += turnSpeed * rx;

            double x1 = desiredAngle - angles.firstAngle;
            double x2;
            if (desiredAngle - angles.firstAngle < 0) {
                x2 = desiredAngle - angles.firstAngle + 360;
            } else {
                x2 = desiredAngle - angles.firstAngle - 360;
            }
            double error = Math.abs(x1) < Math.abs(x2) ? x1 : x2;

            integral += error * time;
            eTime.reset();

            double derivative = (error - previousError) / time;
            double correction = P * error + I * integral + D * derivative;

            previousError = error;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double flPower = (y + x + rx + correction) / denominator;
            double frPower = (y - x + rx + correction) / denominator;
            double blPower = (y - x - rx - correction) / denominator;
            double brPower = (y + x - rx - correction) / denominator;

            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);
        }
    }
}
