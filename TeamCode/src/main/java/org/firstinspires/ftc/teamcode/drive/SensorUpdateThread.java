package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

public class SensorUpdateThread extends Thread {

    private final RobotBase robot;

    public static int THREAD_SLEEP_TIME = 20;
    public SensorUpdateThread(RobotBase robot) {
        this.robot = robot;
    }
    @SuppressWarnings({"InfiniteLoopStatement", "BusyWait"})
    @Override
    public void run() {
        try {
            while(true) {
                robot.updateIntakeSampleState();
                Thread.sleep(THREAD_SLEEP_TIME);
            }
        } catch (InterruptedException e) {
            Log.e("RCActivity", "SensorUpdateThread interrupted");
        }
    }
}
