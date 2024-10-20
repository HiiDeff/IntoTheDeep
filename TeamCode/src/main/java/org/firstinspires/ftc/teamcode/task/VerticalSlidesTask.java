package org.firstinspires.ftc.teamcode.task;
import org.firstinspires.ftc.teamcode.drive.V2RobotBase;

public class VerticalSlidesTask extends TimedTask{

    private V2RobotBase robot;
    private final double targetHeight;
    private final double power;
    private boolean started = false;

    public VerticalSlidesTask(V2RobotBase robot, double height) {
        this(robot, height, 0.9);
    }

    public VerticalSlidesTask(V2RobotBase robot, double height, double power) {
        this(robot, height, power, height < 1 ? 1500 : 1000);
    }

    public VerticalSlidesTask(V2RobotBase robot, double height, double power,
                              int finishTimeMillis) {
        this.robot = robot;
        this.targetHeight = height;
        this.power = power;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            started = true;
            robot.setVerticalSlidesHeight(targetHeight, power);
        }
        if (targetHeight < 0.001 &&
                Math.abs(robot.getVerticalSlidesHeight() - targetHeight) < 0.05) {
            cancel();
            return true;
        }
        return false;
    }

    @Override
    public void cancel() {
        if (targetHeight < 1) {
            robot.setVerticalSlidePower(0.0);
        }
    }

}
