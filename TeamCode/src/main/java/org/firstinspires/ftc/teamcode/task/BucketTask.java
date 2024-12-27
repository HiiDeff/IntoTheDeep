package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * Dual mode speed servo has 90 RPM under 4.8 v and 300 degree max rotation.
 */
@Config
public class BucketTask extends TimedTask {

    private static final int RPM = (int) (60/(0.14*6));
    private static final int MAX_ROTATION_DEGREE = 355;

    private RobotBase robot;
    private Position position;
    private boolean started;

    public BucketTask(RobotBase robot, Position position) {
        this.robot = robot;
        this.position = position;
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            double currPos = robot.bucketServo.getPosition();
            double targetPos = robot.getBucketServoPosition(position);
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));
            robot.bucketProfiler.setTargetPosition(targetPos);
            started = true;
        }
        return false;
    }

    public enum Position {
        CARRY,
        CHAMBER_DELIVER,
        DELIVER,
        FLAT,
        TRANSFER
    }
}



