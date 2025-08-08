package org.firstinspires.ftc.teamcode.auto.defaultauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.auto.conditions.SampleStateCondition;
import org.firstinspires.ftc.teamcode.drive.V308Robot.V308Robot;
import org.firstinspires.ftc.teamcode.task.ConditionalTask;
import org.firstinspires.ftc.teamcode.task.DeliveryArmTask;
import org.firstinspires.ftc.teamcode.task.HangTask;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.IntakeTask;
import org.firstinspires.ftc.teamcode.task.IntakeWithExtensionTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.UnboundedIntakeTask;
import org.firstinspires.ftc.teamcode.task.VerticalSlidesTask;

@Config
public abstract class SamplesAutoPath extends AutoBase {

    public static int verticalSlidesDelay = 500;
    public static int DELIVER_DRIVING_SPEED = 20;

    @Override
    protected Task createStartTask() {
        SeriesTask task = new SeriesTask();
        task.add(new ParallelTask(
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos = getPreloadBucketAlignPos();
                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(60)).build();
                        }
                ),
                new VerticalSlidesTask(robot, 39, 1.0, 1500)
        ));
        task.add(new SeriesTask(
                new DeliveryArmTask(robot, DeliveryArmTask.Position.DELIVER),
                new SleepTask(900)
        ));
        return task;
    }
    @Override
    protected Task createCycleTask() {
        state = AutoState.CYCLE;
        SeriesTask task = new SeriesTask();
        //SAMPLE 1
        task.add(new ParallelTask(
                new ParallelTask(
                        new DeliveryArmTask(robot, DeliveryArmTask.Position.TRANSFER),
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose2d pos = getSample1IntakeAlignPos();
                                    return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(45)).build();
                                }
                        ),
                        new SeriesTask(
                                new SleepTask(verticalSlidesDelay),
                                new VerticalSlidesTask(robot, 0, 0.9, 1000)
                        )
                ),
                new SeriesTask(
                       new SleepTask(getIntakeDelay()),
                       new ParallelTask(
                               new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                               new DeliveryArmTask(robot, DeliveryArmTask.Position.FLAT)
                       ),
                       new IntakeWithExtensionTask(robot, 8, 1000)
                )
        ));

        task.add(new ParallelTask(
                new SeriesTask(
                        new ParallelTask(
                                new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER),
                                new DeliveryArmTask(robot, DeliveryArmTask.Position.TRANSFER),
                                new HorizontalSlidesTask(robot, robot.getHorizontalSlideTransferPos(), 1.0, 1200),
                                new UnboundedIntakeTask(robot, 1.0)
                        ),
                        new ParallelTask(
                                new IntakeTask(robot, 300, -0.8),
                                new HorizontalSlidesTask(robot, robot.getHorizontalSlideTransferPos(), 1.0, 300)
                        ),
                        new SeriesTask(
                                new ConditionalTask(new SampleStateCondition(robot)),
                                new ParallelTask(
                                        new UnboundedIntakeTask(robot, 1.0),
                                        new SleepTask(100),
                                        new HorizontalSlidesTask(robot, robot.getHorizontalSlideTransferPos(), 1.0, 300)
                                ),
                                new IntakeTask(robot,1000,-1.0)
                        ),
                        new ParallelTask(
                                new DeliveryArmTask(robot, DeliveryArmTask.Position.TRANSFER),
                                new VerticalSlidesTask(robot, 39, 1.0, 1500)
                        )
                ),
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
//                            Pose2d pos1 = getSample1BucketAlignPos();
                            Pose2d pos2 = getSample1BucketDeliverPos();
                            return builder
//                                    .strafeToLinearHeading(pos1.position, pos1.heading)
                                    .strafeToLinearHeading(pos2.position, pos2.heading, new TranslationalVelConstraint(DELIVER_DRIVING_SPEED))
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
                new DeliveryArmTask(robot, DeliveryArmTask.Position.DELIVER),
                new SleepTask(900)
        ));
        //SECOND SAMPLE
        task.add(new ParallelTask(
                new DeliveryArmTask(robot, DeliveryArmTask.Position.TRANSFER),
                new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos = getSample2IntakeAlignPos();
                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(45)).build();
                        }
                ),
                new SeriesTask(
                        new SleepTask(verticalSlidesDelay),
                        new VerticalSlidesTask(robot, 0, 0.9, 1000)
                )
        ));
        task.add(new ParallelTask(
                new IntakeWithExtensionTask(robot, 14, 1200)

//                new IntakeTask(robot, 500),
//                new HorizontalSlidesTask(robot, 14.0, 1.0, 1000)//,
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
                                new HorizontalSlidesTask(robot, robot.getHorizontalSlideTransferPos(), 1.0, 1200),
                                new UnboundedIntakeTask(robot, 1.0)
                        ),
                        new ParallelTask(
                                new IntakeTask(robot, 300, -0.8),
                                new HorizontalSlidesTask(robot, robot.getHorizontalSlideTransferPos(), 1.0, 300)
                        ),
                        new SeriesTask(
                                new ConditionalTask(new SampleStateCondition(robot)),
                                new ParallelTask(
                                        new UnboundedIntakeTask(robot, 1.0),
                                        new SleepTask(100),
                                        new HorizontalSlidesTask(robot, robot.getHorizontalSlideTransferPos(), 1.0, 300)
                                ),
                                new IntakeTask(robot,1000,-1.0)
                        ),
                        new ParallelTask(
                                new DeliveryArmTask(robot, DeliveryArmTask.Position.TRANSFER),
                                new VerticalSlidesTask(robot, 39, 1.0, 2000)
                        )
                ),
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
//                            Pose2d pos1 = getSample2BucketAlignPos();
                            Pose2d pos2 = getSample2BucketDeliverPos();
                            return builder
//                                    .strafeToLinearHeading(pos1.position, pos1.heading)
                                    .strafeToLinearHeading(pos2.position, pos2.heading, new TranslationalVelConstraint(DELIVER_DRIVING_SPEED)).build();
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
                new DeliveryArmTask(robot, DeliveryArmTask.Position.DELIVER),
                new SleepTask(900)
        ));
        //THIRD SAMPLE
        task.add(new ParallelTask(
                new DeliveryArmTask(robot, DeliveryArmTask.Position.TRANSFER),
                new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos = getSample3IntakeAlignPos();
                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(45)).build();
                        }
                ),
                new SeriesTask(
                        new SleepTask(verticalSlidesDelay),
                        new VerticalSlidesTask(robot, 0, 0.9, 1000)
                )
        ));
        task.add(new ParallelTask(
                new IntakeWithExtensionTask(robot, 13, 1000)

//                new IntakeTask(robot, 500),
//                new HorizontalSlidesTask(robot, 12.0, 1.0, 1000)//,
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
                                new HorizontalSlidesTask(robot, robot.getHorizontalSlideTransferPos(), 1.0, 1200),
                                new UnboundedIntakeTask(robot, 1.0)
                        ),
                        new ParallelTask(
                                new IntakeTask(robot, 300, -0.8),
                                new HorizontalSlidesTask(robot, robot.getHorizontalSlideTransferPos(), 1.0, 300)
                        ),
                        new SeriesTask(
                                new ConditionalTask(new SampleStateCondition(robot)),
                                new ParallelTask(
                                        new UnboundedIntakeTask(robot, 1.0),
                                        new SleepTask(100),
                                        new HorizontalSlidesTask(robot, robot.getHorizontalSlideTransferPos(), 1.0, 300)
                                ),
                                new IntakeTask(robot,1000,-1.0)
                        ),
                        new ParallelTask(
                                new DeliveryArmTask(robot, DeliveryArmTask.Position.TRANSFER),
                                new VerticalSlidesTask(robot, 39, 1.0, 2000)
                        )
                ),
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
//                            Pose2d pos1 = getSample3BucketAlignPos();
                            Pose2d pos2 = getSample3BucketDeliverPos();
                            return builder
//                                    .strafeToLinearHeading(pos1.position, pos1.heading)
                                    .strafeToLinearHeading(pos2.position, pos2.heading, new TranslationalVelConstraint(DELIVER_DRIVING_SPEED))
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
                new DeliveryArmTask(robot, DeliveryArmTask.Position.DELIVER),
                new SleepTask(900)
        ));
        return task;
    }
    @Override
    protected Task createFinishTask() {
        state = AutoState.FINISH;
        SeriesTask task = new SeriesTask();
        task.add(new ParallelTask(
                new DeliveryArmTask(robot, DeliveryArmTask.Position.TRANSFER),
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos1 = getParkAlignPos();
                            Pose2d pos2 = getParkPos();
//                            return builder
//                                    .setTangent(0)
//                                    .splineToSplineHeading(pos2, Math.PI/2)
//                                    //.strafeToLinearHeading(pos2.position, pos2.heading, new TranslationalVelConstraint(60))
//                                    .build();
                            return builder
                                    .strafeToLinearHeading(pos1.position, pos1.heading, new TranslationalVelConstraint(70))
                                    .strafeToLinearHeading(pos2.position, pos2.heading, new TranslationalVelConstraint(70))
                                    .build();
                        }
                ),
                new SeriesTask(
                        new SleepTask(verticalSlidesDelay),
                        new VerticalSlidesTask(robot, 0, 0.9, 1000)
                ),
                new HorizontalSlidesTask(robot, 0, 1.0,5000),
                new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                new HangTask((V308Robot) robot, HangTask.WhichHang.LEFT, HangTask.Position.VERTICAL),
                new HangTask((V308Robot) robot, HangTask.WhichHang.RIGHT, HangTask.Position.VERTICAL)
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
    protected abstract Pose2d getParkAlignPos();
    protected abstract Pose2d getParkPos();
    protected abstract int getIntakeDelay();
}
