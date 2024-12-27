package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.V3Robot;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * 5-turn torque servo has 40 RPM under 4.8 v and 4.5 turn max rotation.
 */
@Config
public class HangTask extends TimedTask {
    public enum WhichHang {
        LEFT,
        RIGHT
    }

    private static final int RPM = 20; // 40 RPM without load
    private static final int MAX_ROTATION_DEGREE = 1620; // 4.5 turn


    private final V3Robot robot;
    private final Position position;
    private final WhichHang hang;
    private boolean started;

    public static void setHangPosition(V3Robot robot,
            WhichHang placer, Position position) {
        getHangServo(robot, placer).setPosition(robot.getHangTargetPos(placer, position));
    }

    public HangTask(V3Robot robot, WhichHang hang, Position position) {
        this.robot = robot;
        this.hang = hang;
        this.position = position;
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            Servo servo = getHangServo(robot, hang);
            double currPos = servo.getPosition();
            double targetPos = robot.getHangTargetPos(hang, position);
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));
            servo.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    private static Servo getHangServo(V3Robot robot, WhichHang hang) {
        return hang == WhichHang.LEFT ? robot.leftHang : robot.rightHang;
    }

    public enum Position {
        RELEASE,
        HANG,
        INIT
    }
}



