package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

public class Dashboard {
    public static TelemetryPacket packet = new TelemetryPacket();
    static FtcDashboard dashboard = FtcDashboard.getInstance();
    static ElapsedTime dashboardTimer = new ElapsedTime();

    public static void periodic() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
        packet.put("Loop time", dashboardTimer.milliseconds());
        dashboardTimer.reset();
    }

    public void startCameraStream(CameraStreamSource source, int maxFps) {
        dashboard.startCameraStream(source,maxFps);
    }
}