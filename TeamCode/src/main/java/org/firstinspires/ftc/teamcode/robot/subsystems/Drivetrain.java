package org.firstinspires.ftc.teamcode.robot.subsystems;

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
import org.firstinspires.ftc.teamcode.utils.KalmanFilter;

@Config
public class Drivetrain {
    public double previous_error, integral = 0;
    public static double P = 0.04;
    public static double I = 0;
    public static double D = 0;

    public static double filterQ = 0.5;
    public static double filterR = 0.5;
    
    private final Telemetry telemetry;

    private final BNO055IMU imu;
    private final DcMotor fr;
    private final DcMotor rr;
    private final DcMotor fl;
    private final DcMotor rl;

    private double FL_power;
    private double FR_power;
    private double RL_power;
    private double RR_power;

    private final ElapsedTime eTime = new ElapsedTime();

    private double errorAutoTurn, errorHeadingControl;
    private double desiredAngleAutoTurn = 0;
    private double desiredAngleHeadingControl = 0;

    private String turnState = "auto";

    private double denominator;

    private KalmanFilter imuFilter = new KalmanFilter(filterQ, filterR, 3);

    private enum DriveMode {
        DRIVER_CONTROLLED,
        AUTO_CONTROL
    }
    private DriveMode driveState = DriveMode.AUTO_CONTROL;

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

        // TODO: see if I need to delete this
        imu.startAccelerationIntegration(null, null, 1000);

        telemetry.addData(">", "Hardware initialized");
        telemetry.update();
    }

    public void driveRobot(double leftStickY, double leftStickX, double rightStickX, boolean slowMode,
                           boolean autoTurn0, boolean autoTurn90, boolean autoTurn180,
                           boolean autoTurn270) {
        double time = eTime.time();
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double readAngle = angles.firstAngle;

        leftStickX *= 1.1;

        double gyro_radians = readAngle * Math.PI/180;
        double newForward = leftStickY * Math.cos(gyro_radians) + leftStickX * Math.sin(gyro_radians);
        double newStrafe = -leftStickY * Math.sin(gyro_radians) + leftStickX * Math.cos(gyro_radians);

        boolean automaticTurning = false; /*(Math.abs(leftStickY) > 0 || Math.abs(leftStickX) > 0) && Math.abs(rightStickX) == 0;*/
        if (!automaticTurning) {
            desiredAngleHeadingControl = readAngle;
        }

        errorAutoTurn = angleWrap(desiredAngleAutoTurn - readAngle);

        telemetry.addData("Turn Error", errorAutoTurn);

        integral += (errorAutoTurn * time);
        eTime.reset();

        double derivative = (errorAutoTurn - previous_error) / time;
        double rcw = P * -errorAutoTurn + I * integral + D * derivative;

        previous_error = errorAutoTurn;

        telemetry.addData("Read angle", readAngle);
        telemetry.addData("RCW", rcw);
        telemetry.addData("Desired Angle", desiredAngleAutoTurn);
        telemetry.addData("rightStickX", rightStickX);

        switch (driveState) {
            case AUTO_CONTROL:
                if (autoTurn0) {
                    desiredAngleAutoTurn = 0;
                }
                if (autoTurn270) {
                    desiredAngleAutoTurn = 270;
                }
                if (autoTurn180) {
                    desiredAngleAutoTurn = 180;
                }
                if (autoTurn90) {
                    desiredAngleAutoTurn = 90;
                }

                if (rightStickX != 0) {
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
                /*if (!automaticTurning) {*/
                    denominator = Math.max(Math.abs(newForward) + Math.abs(newStrafe) + Math.abs(rightStickX), 1);
                    FL_power = (-newForward + newStrafe + rightStickX) / denominator;
                    RL_power = (-newForward - newStrafe + rightStickX) / denominator;
                    FR_power = (-newForward - newStrafe - rightStickX) / denominator;
                    RR_power = (-newForward + newStrafe - rightStickX) / denominator;
                /*} else {
                    errorHeadingControl = angleWrap(desiredAngleHeadingControl - readAngle);
                    rcw = -errorHeadingControl * P;
                    denominator = Math.max(Math.abs(newForward) + Math.abs(newStrafe) + Math.abs(rcw), 1);
                    FL_power = (-newForward + newStrafe + rcw) / denominator;
                    RL_power = (-newForward - newStrafe + rcw) / denominator;
                    FR_power = (-newForward - newStrafe - rcw) / denominator;
                    RR_power = (-newForward + newStrafe - rcw) / denominator;
                }*/
                if (autoTurn0) {
                    desiredAngleAutoTurn = 0;
                    driveState = DriveMode.AUTO_CONTROL;
                }
                if (autoTurn270) {
                    desiredAngleAutoTurn = 270;
                    driveState = DriveMode.AUTO_CONTROL;
                }
                if (autoTurn180) {
                    desiredAngleAutoTurn = 180;
                    driveState = DriveMode.AUTO_CONTROL;
                }
                if (autoTurn90) {
                    desiredAngleAutoTurn = 90;
                    driveState = DriveMode.AUTO_CONTROL;
                }
                break;
        }

        telemetry.addData("turnState", turnState);

        if (slowMode) {
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

    public double angleWrap(double angle) {
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }

        return angle;
    }
}
