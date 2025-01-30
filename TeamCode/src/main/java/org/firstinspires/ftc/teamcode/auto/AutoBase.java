package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.AutoStates;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.SensorUpdateThread;
import org.firstinspires.ftc.teamcode.task.Task;

@Config
public abstract class AutoBase extends LinearOpMode {

    public static int AA_TOTAL_TIME_MILLIS = 30000;

    public static int AA_NUM_OF_CYCLES = 4;

    public static int INIT_DELAY_TIME = 0;
    public static int AA_TIME_FOR_A_CYCLE = 5000;
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
        while (opModeInInit()) {
            telemetry.addData("initializing", 1);
            telemetry.update();
        }
        Task task = createStartTask();
        robot.setLightColor(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        timer.reset();

        sensorUpdateThread = new SensorUpdateThread(robot);
        sensorUpdateThread.start();

        boolean done = false;
        while (opModeIsActive() && !done && task != null) {
            robot.updateEverything();


            if(task.perform()) {
                if(state == AutoState.INIT){
                    task = createCycleTask();
                    //task = createFinishTask();
                }
                else if(state == AutoState.CYCLE) {
                    task = createFinishTask();
                }
                else {
                    task = null;
                    done = true;
                }
            }
            telemetry.addData("status", "running");
            telemetry.addData("angle", robot.pose.heading.toDouble());
            telemetry.addData("x", robot.pose.position.x);
            telemetry.addData("y", robot.pose.position.y);
            telemetry.update();
        }
        sensorUpdateThread.interrupt();
    }

    private boolean timeToFinish() {
        return timer.milliseconds() > AA_TOTAL_TIME_MILLIS - AA_TIME_FOR_PARK;
    }

    private void resetRobot() {
        robot.autoInit();
        robot.setLightColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    protected abstract RobotBase createRobot(HardwareMap hardwareMap);

    protected abstract Task createStartTask();

    protected abstract Task createFinishTask();

    protected abstract Task createCycleTask();

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
