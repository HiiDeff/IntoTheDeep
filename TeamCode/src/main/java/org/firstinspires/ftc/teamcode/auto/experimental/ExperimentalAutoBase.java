package org.firstinspires.ftc.teamcode.auto.experimental;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.AutoStates;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.SensorUpdateThread;
import org.firstinspires.ftc.teamcode.task.Task;

import java.util.ArrayList;
import java.util.List;

@Config
public abstract class ExperimentalAutoBase extends LinearOpMode {

    public static int AA_TOTAL_TIME_MILLIS = 30000;

    public static int AA_NUM_OF_CYCLES = 4;
    public int CURRENT_CYCLE_NUMBER = 0;

    public static int INIT_DELAY_TIME = 0;
    public static int AA_TIME_FOR_A_CYCLE = 4000;
    public static int AA_TIME_FOR_PARK = 1000;

    // Moving velocity constraints
    public static double HIGH_SPEED = 35;
    public static double LOW_SPEED = 30;

    protected final ElapsedTime timer = new ElapsedTime();
    protected RobotBase robot;

    protected AutoState state = AutoState.INIT;
    protected AutoStates autoStates = new AutoStates();
    protected SensorUpdateThread sensorUpdateThread;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = createRobot(hardwareMap);
        resetRobot();


        Task startTask = createStartTask();
        List<Task> cycleTasks = new ArrayList<Task>();
        for(int i=0; i<AA_NUM_OF_CYCLES; i++){
            cycleTasks.add(createCycleTask(i));
        }
        Task finishTask = createFinishTask();

        Task currentTask = startTask;
        CURRENT_CYCLE_NUMBER = 0;


        while (opModeInInit()) {
            telemetry.addData("initializing", 1);
            telemetry.update();
        }

        sensorUpdateThread = new SensorUpdateThread(robot);
        sensorUpdateThread.start();

        robot.setLightColor(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        timer.reset();

        boolean done = false;
        while (opModeIsActive() && !done && currentTask != null) {
            robot.updateEverything();

            if(timeToFinish() && state != AutoState.FINISH) {
                currentTask.cancel();
                currentTask = finishTask;
            } else if(currentTask.perform()) {
                if(hasTimeForOneMoreCycle()) {
                    currentTask = cycleTasks.get(CURRENT_CYCLE_NUMBER);
                    CURRENT_CYCLE_NUMBER++;
                } else if(state != AutoState.FINISH) {
                    currentTask = finishTask;
                } else{
                    done = true;
                }
            }

            telemetry.addData("status", "running");
            telemetry.addData("angle", robot.pose.heading.toDouble());
            telemetry.addData("x", robot.pose.position.x);
            telemetry.addData("y", robot.pose.position.y);
            telemetry.update();
        }
    }

    private boolean timeToFinish() {
        return false;
        //return timer.milliseconds() > AA_TOTAL_TIME_MILLIS - AA_TIME_FOR_PARK;
    }

    private boolean hasTimeForOneMoreCycle() {
        // Go ahead if there are still 10 seconds left and
        return CURRENT_CYCLE_NUMBER < AA_NUM_OF_CYCLES; // && timer.milliseconds() < AA_TOTAL_TIME_MILLIS - AA_TIME_FOR_A_CYCLE;
    }

    private void resetRobot() {
        robot.autoClipInit();
        robot.setLightColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    protected abstract RobotBase createRobot(HardwareMap hardwareMap);

    protected abstract Task createStartTask();

    protected abstract Task createFinishTask();

    protected abstract Task createCycleTask(int cycleNumber);

    protected double toR(double degree) {
        return Math.toRadians(degree);
    }

    // Reverse the direction and convert the degree to radius.
    protected double toOppositeR(double degree) {
        return Math.toRadians(degree + 180);
    }

    protected enum AutoState {
        INIT,
        FINISH,
        CYCLE,
    }
}
