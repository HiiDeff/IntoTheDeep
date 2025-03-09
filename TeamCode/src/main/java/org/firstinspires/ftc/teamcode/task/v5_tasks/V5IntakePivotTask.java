package org.firstinspires.ftc.teamcode.task.v5_tasks;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.V5Robot.V5Robot;
import org.firstinspires.ftc.teamcode.task.TimedTask;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * Dual mode speed servo has 90 RPM under 4.8 v and 300 degree max rotation.
 */
@Config
public class V5IntakePivotTask extends TimedTask {

    private static final int RPM = 50;
    private static final int MAX_ROTATION_DEGREE = 300;

    private V5Robot robot;
    private Position position;
    private double exactPosition;
    private boolean started;

    public V5IntakePivotTask(V5Robot robot, Position position) {
        this.robot = robot;
        this.position = position;
    }
    public V5IntakePivotTask(V5Robot robot, double exactPosition) {
        this.robot = robot;
        this.exactPosition = exactPosition;
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            Servo servo = robot.intakePivot;
            double currPos = servo.getPosition();
            double targetPos = position!=null ? robot.getIntakePivotPosition(position):exactPosition;
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));
            servo.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    public enum Position {
        VERTICAL,
        HORIZONTAL,
        TILTED_LEFT,
        TILTED_RIGHT
    }
}



