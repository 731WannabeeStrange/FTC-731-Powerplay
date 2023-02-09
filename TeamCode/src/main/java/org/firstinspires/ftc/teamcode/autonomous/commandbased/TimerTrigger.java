package org.firstinspires.ftc.teamcode.autonomous.commandbased;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.button.Trigger;

@Config
public class TimerTrigger extends Trigger {
    public static double parkingBuffer = 5;
    private final double startTime;
    private double currentTime;

    public TimerTrigger(double time) {
        startTime = time;
    }

    @Override
    public boolean get() {
        return currentTime - startTime > 30 - parkingBuffer;
    }

    public void update(double runTime) {
        currentTime = runTime;
    }
}