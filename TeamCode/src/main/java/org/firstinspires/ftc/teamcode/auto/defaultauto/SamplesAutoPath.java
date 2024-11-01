package org.firstinspires.ftc.teamcode.auto.defaultauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.drive.V2RobotBase;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.IntakeReverseTask;
import org.firstinspires.ftc.teamcode.task.IntakeTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.UnboundedIntakeReverseTask;
import org.firstinspires.ftc.teamcode.task.UnboundedIntakeTask;
import org.firstinspires.ftc.teamcode.task.UnboundedSeriesTask;
import org.firstinspires.ftc.teamcode.task.V2DiffyTask;
import org.firstinspires.ftc.teamcode.task.VerticalSlidesTask;

@Config
public abstract class SamplesAutoPath extends AutoBase {

    @Override
    protected Task createStartTask() {
        SeriesTask task = new SeriesTask();
        task.add(new ParallelTask(
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos = getPreloadBucketAlignPos();
                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(45)).build();
                        }
                ),
                new VerticalSlidesTask(robot, 39, 1.0, 2000)
        ));
        task.add(new SeriesTask(
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos = getPreloadBucketDeliverPos();
                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(45)).build();
                        }
                ),
                new ParallelTask(
                        new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.LEFT, V2DiffyTask.DiffyPosition.DELIVER),
                        new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.RIGHT, V2DiffyTask.DiffyPosition.DELIVER)
                ),
                new SleepTask(500)
        ));
        task.add(new ParallelTask(
                new ParallelTask(
                        new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.LEFT, V2DiffyTask.DiffyPosition.TRANSFER),
                        new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.RIGHT, V2DiffyTask.DiffyPosition.TRANSFER)
                ),
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos = getSample1IntakeAlignPos();
                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(45)).build();
                        }
                ),
                new VerticalSlidesTask(robot, 0, 0.9, 3000)
        ));
        return task;
    }
    @Override
    protected Task createCycleTask() {
        state = AutoState.CYCLE;
        SeriesTask task = new SeriesTask();
        //SAMPLE 1
        task.add(new ParallelTask(
                new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                new IntakeTask(robot, 500),
                new HorizontalSlidesTask(robot, 8.0, 1.0, 1000)//,
//                new RuntimeDrivingTask(
//                        robot,
//                        builder -> {
//                            Pose2d pos = getSample1IntakePos();
//                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(35)).build();
//                        }
//                )
        ));
        task.add(new ParallelTask(
                new SeriesTask(
                        new ParallelTask(
                                new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER),
                                new HorizontalSlidesTask(robot, V2RobotBase.HORIZONTAL_SLIDE_TRANSFER_POS, 1.0, 1500)
                        ),
                        new SleepTask(300),
                        new IntakeReverseTask(robot, 800),
                        new ParallelTask(
                                new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.LEFT, V2DiffyTask.DiffyPosition.CARRY),
                                new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.RIGHT, V2DiffyTask.DiffyPosition.CARRY),
                                new VerticalSlidesTask(robot, 39, 1.0, 2000)
                        )
                ),
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos1 = getSample1BucketAlignPos();
                            Pose2d pos2 = getSample1BucketDeliverPos();
                            return builder
                                    .strafeToLinearHeading(pos1.position, pos1.heading)
                                    .strafeToLinearHeading(pos2.position, pos2.heading)
                                    .build();
                        }
                )
        ));
        task.add(new SeriesTask(
//                new RuntimeDrivingTask(
//                        robot,
//                        builder -> {
//                            Pose2d pos = getSample1BucketDeliverPos();
//                            return builder.strafeToLinearHeading(pos.position, pos.heading).build();
//                        }
//                ),
                new ParallelTask(
                        new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.LEFT, V2DiffyTask.DiffyPosition.DELIVER),
                        new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.RIGHT, V2DiffyTask.DiffyPosition.DELIVER)
                ),
                new SleepTask(500)
        ));
        //SECOND SAMPLE
        task.add(new ParallelTask(
                new ParallelTask(
                        new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.LEFT, V2DiffyTask.DiffyPosition.TRANSFER),
                        new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.RIGHT, V2DiffyTask.DiffyPosition.TRANSFER)
                ),
                new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos = getSample2IntakeAlignPos();
                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(45)).build();
                        }
                ),
                new VerticalSlidesTask(robot, 0, 0.9, 1000)
        ));
        task.add(new ParallelTask(
                new IntakeTask(robot, 500),
                new HorizontalSlidesTask(robot, 14.0, 1.0, 1000)//,
//                new RuntimeDrivingTask(
//                        robot,
//                        builder -> {
//                            Pose2d pos = getSample2IntakePos();
//                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(35)).build();
//                        }
//                )
        ));
        task.add(new ParallelTask(
                new SeriesTask(
                        new ParallelTask(
                                new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER),
                                new HorizontalSlidesTask(robot, V2RobotBase.HORIZONTAL_SLIDE_TRANSFER_POS, 1.0, 1500)
                        ),
                        new SleepTask(300),
                        new IntakeReverseTask(robot, 800),
                        new ParallelTask(
                                new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.LEFT, V2DiffyTask.DiffyPosition.CARRY),
                                new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.RIGHT, V2DiffyTask.DiffyPosition.CARRY),
                                new VerticalSlidesTask(robot, 39, 1.0, 2000)
                        )
                ),
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos1 = getSample2BucketAlignPos();
                            Pose2d pos2 = getSample2BucketDeliverPos();
                            return builder
                                    .strafeToLinearHeading(pos1.position, pos1.heading)
                                    .strafeToLinearHeading(pos2.position, pos2.heading).build();
                        }
                )
        ));
        task.add(new SeriesTask(
//                new RuntimeDrivingTask(
//                        robot,
//                        builder -> {
//                            Pose2d pos = getSample2BucketDeliverPos();
//                            return builder.strafeToLinearHeading(pos.position, pos.heading).build();
//                        }
//                ),
                new ParallelTask(
                        new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.LEFT, V2DiffyTask.DiffyPosition.DELIVER),
                        new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.RIGHT, V2DiffyTask.DiffyPosition.DELIVER)
                ),
                new SleepTask(500)
        ));
        //THIRD SAMPLE
        task.add(new ParallelTask(
                new ParallelTask(
                        new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.LEFT, V2DiffyTask.DiffyPosition.TRANSFER),
                        new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.RIGHT, V2DiffyTask.DiffyPosition.TRANSFER)
                ),
                new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos = getSample3IntakeAlignPos();
                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(45)).build();
                        }
                ),
                new VerticalSlidesTask(robot, 0, 0.9, 1000)
        ));
        task.add(new ParallelTask(
                new IntakeTask(robot, 500),
                new HorizontalSlidesTask(robot, 14.0, 1.0, 1000)//,
//                new RuntimeDrivingTask(
//                        robot,
//                        builder -> {
//                            Pose2d pos = getSample3IntakePos();
//                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(35)).build();
//                        }
//                )
        ));
        task.add(new ParallelTask(
                new SeriesTask(
                        new ParallelTask(
                                new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER),
                                new HorizontalSlidesTask(robot, V2RobotBase.HORIZONTAL_SLIDE_TRANSFER_POS, 1.0, 1500)
                        ),
                        new SleepTask(300),
                        new IntakeReverseTask(robot, 800),
                        new ParallelTask(
                                new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.LEFT, V2DiffyTask.DiffyPosition.CARRY),
                                new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.RIGHT, V2DiffyTask.DiffyPosition.CARRY),
                                new VerticalSlidesTask(robot, 39, 1.0, 2000)
                        )
                ),
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos1 = getSample3BucketAlignPos();
                            Pose2d pos2 = getSample3BucketDeliverPos();
                            return builder
                                    .strafeToLinearHeading(pos1.position, pos1.heading)
                                    .strafeToLinearHeading(pos2.position, pos2.heading)
                                    .build();
                        }
                )
        ));
        task.add(new SeriesTask(
//                new RuntimeDrivingTask(
//                        robot,
//                        builder -> {
//                            Pose2d pos = getSample3BucketDeliverPos();
//                            return builder.strafeToLinearHeading(pos.position, pos.heading).build();
//                        }
//                ),
                new ParallelTask(
                        new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.LEFT, V2DiffyTask.DiffyPosition.DELIVER),
                        new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.RIGHT, V2DiffyTask.DiffyPosition.DELIVER)
                ),
                new SleepTask(500)
        ));
        return task;
    }
    @Override
    protected Task createFinishTask() {
        state = AutoState.FINISH;
        SeriesTask task = new SeriesTask();
        task.add(new ParallelTask(
                new ParallelTask(
                        new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.LEFT, V2DiffyTask.DiffyPosition.TRANSFER),
                        new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.RIGHT, V2DiffyTask.DiffyPosition.TRANSFER)
                ),
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos = getParkPos();
                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(45)).build();
                        }
                ),
                new VerticalSlidesTask(robot, 0, 0.9, 3000),
                new HorizontalSlidesTask(robot, 0, 1.0,1500),
                new IntakeArmTask(robot, IntakeArmTask.Position.DOWN)
        ));
        return task;
    }

    protected abstract Pose2d getPreloadBucketAlignPos();
    protected abstract Pose2d getPreloadBucketDeliverPos();
    protected abstract Pose2d getSample1IntakeAlignPos();

    protected abstract Pose2d getSample1IntakePos();
    protected abstract Pose2d getSample1BucketAlignPos();
    protected abstract Pose2d getSample1BucketDeliverPos();
    protected abstract Pose2d getSample2IntakeAlignPos();

    protected abstract Pose2d getSample2IntakePos();
    protected abstract Pose2d getSample2BucketAlignPos();
    protected abstract Pose2d getSample2BucketDeliverPos();
    protected abstract Pose2d getSample3IntakeAlignPos();
    protected abstract Pose2d getSample3IntakePos();
    protected abstract Pose2d getSample3BucketAlignPos();
    protected abstract Pose2d getSample3BucketDeliverPos();
    protected abstract Pose2d getParkPos();

}
