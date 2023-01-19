package org.firstinspires.ftc.teamcode.autonomous.roadrunner.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.autonomous.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

public class T265Localizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 0;
    public static double WHEEL_RADIUS = 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = 0; // X is the up and down direction
    public static double PARALLEL_Y = 0; // Y is the strafe direction

    public static double PERPENDICULAR_X = 0;
    public static double PERPENDICULAR_Y = 0;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;
    private Pose2d poseOffset;
    private Pose2d mPoseEstimate;
    private Pose2d rawPose;
    private T265Camera.CameraUpdate cameraUpdate;

    public static T265Camera slamra;
    private T265Camera.PoseConfidence poseConfidence;

    public T265Localizer(HardwareMap hardwareMap, boolean resetPos) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelEncoder"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicularEncoder"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        poseOffset = new Pose2d();
        mPoseEstimate = new Pose2d();
        rawPose = new Pose2d();

        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(new Translation2d(0,0), new Rotation2d(0)), 0.5, hardwareMap.appContext);
            System.out.println("731: Created Realsense object");
            setPoseEstimate(new Pose2d(0,0,0));
        }
        try {
            slamra.start();
            System.out.println("731: Started Realsense camera");
        } catch (Exception ignored) {
            System.out.println("731: Realsense already started");
            if (resetPos) {
                slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(0,0, new Rotation2d(0)));
            }
        }
        if (slamra.getLastReceivedCameraUpdate().confidence == T265Camera.PoseConfidence.Failed) {
            System.out.println("731: Realsense failed to get position");
        }
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        if (cameraUpdate != null) {
            Translation2d oldPose = cameraUpdate.pose.getTranslation();
            Rotation2d oldRot = cameraUpdate.pose.getRotation();
            //The T265's unit of measurement is meters.  dividing it by .0254 converts meters to inches.
            rawPose = new Pose2d(oldPose.getX() / .0254, oldPose.getY() / .0254, Angle.norm(oldRot.getRadians())); //raw pos
            mPoseEstimate = rawPose.plus(poseOffset); //offsets the pose to be what the pose estimate is;
        } else {
            System.out.println("731: Null camera update");
        }

        return mPoseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        super.setPoseEstimate(pose2d);

        pose2d = new Pose2d(pose2d.getX(),pose2d.getY(),0);
        System.out.println("731: Setting pose estimate to " + pose2d.toString());
        poseOffset = pose2d.minus(rawPose);
        poseOffset = new Pose2d(poseOffset.getX(), poseOffset.getY(), Math.toRadians(0));
        System.out.println("731: Set pose offset to " + poseOffset.toString());
    }

    public T265Camera.PoseConfidence getConfidence() {
        return poseConfidence;
    }

    public double getHeading() {
        return Angle.norm(mPoseEstimate.getHeading());
    }

    public Double getHeadingVelocity() { return cameraUpdate.velocity.omegaRadiansPerSecond; }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        ChassisSpeeds velocity = cameraUpdate.velocity;
        return new Pose2d(velocity.vxMetersPerSecond /.0254,velocity.vyMetersPerSecond /.0254,velocity.omegaRadiansPerSecond);
    }

    @Override
    public void update() {
        super.update();
        Pose2d odometryVelocities = super.getPoseVelocity();

        // Multiply by 0.0254 to convert inches to meters for ftc265 library
        slamra.sendOdometry(odometryVelocities.getX() * 0.0254, odometryVelocities.getY() * 0.0254);

        cameraUpdate = slamra.getLastReceivedCameraUpdate();
        poseConfidence = cameraUpdate.confidence;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()),
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()),
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity())
        );
    }
}
