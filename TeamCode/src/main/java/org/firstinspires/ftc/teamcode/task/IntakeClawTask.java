package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

public class IntakeClawTask extends TimedTask{

    private RobotBase robot;
    private boolean started = false;

    // TODO: Andrew please tune me
    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;
    private ClawPosition position;

    public IntakeClawTask(RobotBase robot, ClawPosition position){
        this.robot = robot;
        this.position = position;
    }

    @Override
    public boolean performInternal(){
        if (!started){
            double currPos = robot.intakeClaw.getPosition(); // where is the intake claw servo supposed to be
            double targetPos = robot.getIntakeClawPosition(position);

            // no idea how estimating servo time works but it is very important
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE, targetPos -currPos));

            robot.intakeClaw.setPosition(targetPos);
            // alternatively: robot.setIntakeClawPosition(position);

            started = true;
        }
        return false;
    }

    public enum ClawPosition{
        OPEN,
        CLOSE
    }

}
