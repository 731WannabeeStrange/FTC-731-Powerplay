package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
public class Drivetrain {
    private final Telemetry telemetry;

    private final BNO055IMU imu;
    private final DcMotor fr;
    private final DcMotor rr;
    private final DcMotor fl;
    private final DcMotor rl;

    private final ElapsedTime eTime = new ElapsedTime();

    private double previous_error, integral = 0;
    public static double P = 0.04;
    public static double I = 0;
    public static double D = 0;

    public Drivetrain(HardwareMap hardwareMap, Telemetry multipleTelemetry) {
        telemetry = multipleTelemetry;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample op mode
        parameters.mode = BNO055IMU.SensorMode.IMU;

        fr = hardwareMap.get(DcMotor.class, "frMotor");
        rr = hardwareMap.get(DcMotor.class, "rrMotor");
        fl = hardwareMap.get(DcMotor.class, "flMotor");
        rl = hardwareMap.get(DcMotor.class, "rlMotor");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotor.Direction.REVERSE);
        rl.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(null, null, 1000);

        telemetry.addData(">", "Hardware initialized");
        telemetry.update();
    }

    public void driveRobot(double leftStickY, double leftStickX, double rightStickX, boolean leftBumper, boolean autoTurn, double desiredAngle) {
        double time = eTime.time();
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        leftStickX *= 1.1;

        double gyro_radians = angles.firstAngle * Math.PI/180;
        double newForward = leftStickY * Math.cos(gyro_radians) + leftStickX * Math.sin(gyro_radians);
        double newStrafe = -leftStickY * Math.sin(gyro_radians) + leftStickX * Math.cos(gyro_radians);

        double x1 = desiredAngle - angles.firstAngle;
        double x2;
        if (x1 < 0) {
            x2 = x1 + 360;
        } else {
            x2 = x1 - 360;
        }
        double error = Math.abs(x1) < Math.abs(x2) ? x1 : x2;

        telemetry.addData("Turn Error", error);

        integral += (error * time);
        eTime.reset();

        double derivative = (error - previous_error) / time;
        double rcw = P * -error + I * integral + D * derivative;

        previous_error = error;

        telemetry.addData("Read angle", angles.firstAngle);
        telemetry.addData("RCW", rcw);
        telemetry.addData("Desired Angle", desiredAngle);

        double FL_power, RL_power, FR_power, RR_power;

        if (autoTurn) {
            //denominator = Math.max(Math.abs(newForward) + Math.abs(newStrafe) + Math.abs(rcw), 1);
            FL_power = (-newForward + newStrafe + rcw);// / denominator;
            RL_power = (-newForward - newStrafe + rcw);// / denominator;
            FR_power = (-newForward - newStrafe - rcw);// / denominator;
            RR_power = (-newForward + newStrafe - rcw);// / denominator;
        } else {
            double denominator = Math.max(Math.abs(newForward) + Math.abs(newStrafe) + Math.abs(rightStickX), 1);
            FL_power = (-newForward + newStrafe + rightStickX) / denominator;
            RL_power = (-newForward - newStrafe + rightStickX) / denominator;
            FR_power = (-newForward - newStrafe - rightStickX) / denominator;
            RR_power = (-newForward + newStrafe - rightStickX) / denominator;
        }

        if (leftBumper) {
            FL_power /= 4;
            FR_power /= 4;
            RL_power /= 4;
            RR_power /= 4;
        }

        telemetry.addData("FLpower", FL_power);
        telemetry.addData("FRpower", FR_power);
        telemetry.addData("RLpower", RL_power);
        telemetry.addData("RRpower", RR_power);

        fl.setPower(FL_power);
        fr.setPower(FR_power);
        rl.setPower(RL_power);
        rr.setPower(RR_power);
    }

    public void driveRobotSimple(double y, double x, double rx) {
        y *= -1;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flPower = (y + x + rx) / denominator;
        double frPower = (y - x + rx) / denominator;
        double rlPower = (y - x - rx) / denominator;
        double rrPower = (y + x - rx) / denominator;

        fl.setPower(flPower);
        fr.setPower(frPower);
        rl.setPower(rlPower);
        rr.setPower(rrPower);
    }
}
