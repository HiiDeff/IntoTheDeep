package org.firstinspires.ftc.teamcode.auto.conditions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.Condition;
import org.firstinspires.ftc.teamcode.drive.V2RobotBase;

@Config
public class SampleStateCondition implements Condition {

    private final V2RobotBase robot;

    public SampleStateCondition(V2RobotBase robot) {
        this.robot = robot;
    }
    @Override
    public boolean shouldContinue() {
        return robot.intakeSampleState.IsSampleIn();
    }
}
