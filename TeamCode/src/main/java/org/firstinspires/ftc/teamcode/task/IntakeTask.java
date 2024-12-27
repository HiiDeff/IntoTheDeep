package org.firstinspires.ftc.teamcode.task;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class IntakeTask extends TimedTask {

    private RobotBase robot;
    private boolean started = false;
    private double intakeSpeed;
    private ElapsedTime elapsedTime;

    public IntakeTask(RobotBase robot, int finishTimeMillis, double intakeSpeed) {
        this.robot = robot;
        this.intakeSpeed = intakeSpeed;
        setFinishTimeMillis((finishTimeMillis));
    }

    @Override
    protected boolean performInternal() {
        if(!started) {
            started = true;
            robot.runIntakeWithPower(intakeSpeed);
        } else{
            if((intakeSpeed<0) ^ (robot.intakeSampleState.IsSampleIn())) {
                if(elapsedTime == null){
                    elapsedTime = new ElapsedTime();
                } else if(elapsedTime.milliseconds() > 50) {
                    cancel();
                    return true;
                }
            }
        }
        return false;
    }

    @Override
    public void cancel() {
        robot.stopIntake();
    }
}
