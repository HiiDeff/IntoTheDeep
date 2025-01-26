package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.common.Condition;

/**
 * A task keeps running till the connection is met. Can be used in both SeriesTask and ParallelTask.
 */
public class RunTillTask implements Task {
    private final Condition condition;

    public RunTillTask(Condition condition) {
        this.condition = condition;
    }

    @Override
    public boolean perform() {
        return condition.shouldContinue();
    }
}
