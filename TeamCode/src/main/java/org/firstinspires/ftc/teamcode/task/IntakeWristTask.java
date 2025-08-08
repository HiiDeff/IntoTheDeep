package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

public class IntakeWristTask extends TimedTask{

    private RobotBase robot;
    private boolean started = false;

    // TODO: Andrew please tune me
    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;
    private WristPosition position;

    public IntakeWristTask(RobotBase robot, WristPosition position){
        this.robot = robot;
        this.position = position;
    }

    @Override
    public boolean performInternal(){
        if (!started){
            double currPos = robot.intakeWrist.getPosition();
            double targetPos = robot.getIntakeWristPosition(position);

            // no idea how estimating servo time works but it is very important
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE, targetPos -currPos));

            robot.intakeWrist.setPosition(targetPos);

            started = true;
        }
        return false;
    }

    public enum WristPosition{
        DOWN,
        TRANSFER
    }

}
