package org.firstinspires.ftc.teamcode.task;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.V2RobotBase;

public class IntakeReverseTask extends TimedTask {

    private V2RobotBase robot;
    private boolean started = false;
    private double intakeSpeed;
    private ElapsedTime elapsedTime;


    public IntakeReverseTask(V2RobotBase robot, int finishTimeMillis, double intakeSpeed) {
        this.robot = robot;
        this.intakeSpeed = intakeSpeed;
        setFinishTimeMillis((finishTimeMillis));
    }

    @Override
    protected boolean performInternal() {
        if(!started) {
            started = true;
            robot.runIntakeWithPower(intakeSpeed);
        } else {
            if(!robot.intakeSampleState.IsSampleIn()) {
                if(elapsedTime == null) {
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
