package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

public class SensorUpdateThread extends Thread {

    private final RobotBase robot;

    public SensorUpdateThread(RobotBase robot) {
        this.robot = robot;
    }
    @SuppressWarnings({"InfiniteLoopStatement", "BusyWait"})
    @Override
    public void run() {
        try {
            while(true) {
                robot.updateIntakeSampleState();
                Thread.sleep(100);
            }
        } catch (InterruptedException e) {
            Log.e("RCActivity", "AutoTransitioner shutdown, thread interrupted");
        }
    }
}
