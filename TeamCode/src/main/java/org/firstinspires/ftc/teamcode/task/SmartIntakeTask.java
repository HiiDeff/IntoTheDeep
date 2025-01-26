package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.SampleColor;

/**
 * Dual mode speed servo has 90 RPM under 4.8 v and 300 degree max rotation.
 */
@Config
public class SmartIntakeTask implements Task {

    public static int HORIZONTAL_RETRACTION_MAX_MILLIS = 1000;
    public static double OUTTAKE_POWER_RATIO = 0.4;

    private final RobotBase robot;
    private final SampleColor targetColor;
    private final double power;
    private State state = State.INIT;
    private Task task;

    public SmartIntakeTask(RobotBase robot, SampleColor targetColor, double power) {
        this.robot = robot;
        this.targetColor = targetColor;
        this.power = power;
    }

    @Override
    public boolean perform() {
        switch (state) {
            case INIT:
                task = createIntakeTask();
                state = State.INTAKE;
                break;
            case INTAKE:
                if (task.perform()) {
                    if (isColorCorrect(robot.getSampleColor())) {
                        task = createFinishTask();
                    } else {
                        robot.stopIntake();
                        task = createEmittingTask();
                    }
                    state = State.FINISHING;
                }
                break;
            case FINISHING:
                if (task.perform()) {
                    robot.stopIntake();
                    state = State.DONE;
                }
                break;
            case DONE:
            default:
                break;
        }
        return state == State.DONE;
    }

    private boolean isColorCorrect(SampleColor color) {
        // When the color us unknown, we treat it as correct color to be conservative.
        if (targetColor == SampleColor.COLOR_ALL || color == SampleColor.COLOR_UNSPECIFIED)
            return true;

        switch (targetColor) {
            case COLOR_NOT_BLUE:
                return color != SampleColor.COLOR_BLUE;
            case COLOR_NOT_RED:
                return color != SampleColor.COLOR_RED;
            default:
                return color == targetColor;
        }
    }

    private Task createIntakeTask() {
        return new ParallelTask(
                new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                new UnboundedIntakeTask(robot, power),
                new RunTillTask(robot::isSampleIn));
    }

    private Task createEmittingTask() {
        return new SeriesTask(
                new IntakeArmTask(robot, IntakeArmTask.Position.SLIGHTLY_UP),
                new ParallelTask(
                        new UnboundedIntakeTask(robot, -power * OUTTAKE_POWER_RATIO),
                        new SleepTask(500),
                        new RunTillTask(() -> !robot.isSampleIn())
                )
        );
    }

    private Task createFinishTask() {
        return new ParallelTask(
                new IntakeArmTask(robot, IntakeArmTask.Position.UP),
                new HorizontalSlidesTask(robot, 0.0, 1.0, HORIZONTAL_RETRACTION_MAX_MILLIS));
    }


    public void cancel() {
        if (task != null) {
            task.cancel();
            task = null;
        }
        state = State.DONE;
    }

    private enum State {
        INIT,
        INTAKE,
        FINISHING,
        DONE
    }
}



