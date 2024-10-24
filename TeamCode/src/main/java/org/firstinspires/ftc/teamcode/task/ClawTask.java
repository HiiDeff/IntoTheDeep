package org.firstinspires.ftc.teamcode.task;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.V2Robot;
import org.firstinspires.ftc.teamcode.drive.V2RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

public class ClawTask implements Task {

    private final V2RobotBase robot;
    private boolean started;
    private final ClawPosition position;

    public ClawTask(V2RobotBase robot, ClawPosition position) {
        this.robot = robot;
        this.position = position;
    }

    @Override
    public boolean perform() {
//        if (!started) {
//            Servo servo = robot.intakeArm;
//            double currPos = servo.getPosition();
//            double targetPos = getTargetPos(position);
//            // Only consider the half of normal time since we don't have to wait for the servo to
//            // finish.
//            setFinishTimeMillis(Utils.getEstimatedServoTime(V2Robot.INTAKE_ARM_RPM, V2Robot.INTAKE_ARM_RANGE,
//                    targetPos - currPos) / 2);
//            servo.setPosition(targetPos);
//            started = true;
//        }
        return false;
    }

    public enum ClawPosition {
        OPEN,
        CLOSE
    }
}
