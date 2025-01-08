package org.firstinspires.ftc.teamcode.auto.defaultauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;

import org.firstinspires.ftc.teamcode.auto.NewAutoBase;
import org.firstinspires.ftc.teamcode.auto.conditions.SampleStateCondition;
import org.firstinspires.ftc.teamcode.task.BucketTask;
import org.firstinspires.ftc.teamcode.task.ClawTask;
import org.firstinspires.ftc.teamcode.task.ConditionalTask;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.IntakeTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.TimedHorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.UnboundedIntakeTask;
import org.firstinspires.ftc.teamcode.task.VerticalSlidesTask;

@Config
public abstract class ClipAutoPath extends NewAutoBase {

    public static int verticalSlidesDelay = 500;
    public static double PRELOAD_CHAMBER_HEIGHT = 20.5, DELIVERY_CHAMBER_HEIGHT = 21;
    public static int CLAW_OPEN_DELAY = 420; //18.2745;
    public static int CYCLE_RETURN_SPEED = 35;

    @Override
    protected Task createStartTask() {
        SeriesTask task = new SeriesTask();
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(
                            robot,
                            builder -> {
                                Pose2d pos = getPreloadSubmersibleDeliverPos();
                                return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(60)).build();
                            }
                        ),
                        new VerticalSlidesTask(robot, PRELOAD_CHAMBER_HEIGHT, 1.0, 1000),
                        new SeriesTask(
                                new SleepTask(500),
                                new BucketTask(robot, BucketTask.Position.CHAMBER_DELIVER)
                        ),
                        new SeriesTask(
                                new SleepTask(200),
                                new IntakeArmTask(robot, IntakeArmTask.Position.DOWN)
                        )
                )
        );
//        task.add(new SleepTask(500));
        task.add(
                new ParallelTask(
                        new VerticalSlidesTask(robot, 0, 1.0, 1000),
                        new SeriesTask(
                                new SleepTask(1000),
                                new BucketTask(robot, BucketTask.Position.FLAT)
                        ),
                        new SeriesTask(
                                new SleepTask(CLAW_OPEN_DELAY),
                                new ClawTask(robot, ClawTask.ClawPosition.OPEN)
                        ),
                        new SeriesTask(
                                new SleepTask(100),
                                new RuntimeDrivingTask(
                                        robot,
                                        builder -> {
                                            Pose2d pos1 = getFirstSampleIntakePos();
                                            return builder
                                                    .strafeToLinearHeading(pos1.position, pos1.heading, new TranslationalVelConstraint(40))
                                                    .build();
                                        }
                                )
                        ),
                        new SeriesTask(
                                new SleepTask(800), //TODO: save time here
                                new HorizontalSlidesTask(robot, 22.0, 1.0, 600) //TODO: 300?
                        ),
                        new UnboundedIntakeTask(robot, 1.0)
                )
        );
//        task.add(new SleepTask(1000));
        task.add(
                new ParallelTask(
                        new IntakeArmTask(robot, IntakeArmTask.Position.SLIGHTLY_UP),
                        new RuntimeDrivingTask(
                            robot,
                            builder -> {
                                Pose2d pos1 = getFirstSampleOuttakePos();
                                return builder
                                        .turnTo(pos1.heading, new TurnConstraints(4*Math.PI, -Math.PI*4, Math.PI*4)) //TODO: 60? // new TurnConstraints(4*Math.PI, -Math.PI*4, Math.PI*4))
                                        .build();
                            }
                        ),
                        new HorizontalSlidesTask(robot, 21.0, 1.0, 300),
                        new SeriesTask(
                                new ParallelTask(
                                        new UnboundedIntakeTask(robot, 0.5),
                                        new SleepTask(600)
                                ),
                                new ParallelTask(
                                        new UnboundedIntakeTask(robot, -1.0),
                                        new SleepTask(200)
                                )
                        )
                )
        );
        task.add(
                new SeriesTask(
                        new ConditionalTask(new SampleStateCondition(robot)),
                        new ParallelTask(
                                new UnboundedIntakeTask(robot, 1.0),
                                new SleepTask(200)
                        ),
                        new IntakeTask(robot,400,-1.0)
                )
        );
//        new SleepTask(1000);
        task.add(
                new ParallelTask(
                        new UnboundedIntakeTask(robot, 1.0),
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose2d pos1 = getSecondSampleIntakePos();
                                    return builder
                                            .strafeToLinearHeading(pos1.position, pos1.heading, new TranslationalVelConstraint(40))
                                            .build();
                                }
                        ),
                        new SeriesTask(
                                new TimedHorizontalSlidesTask(robot, 10, 1.0, 500),
                                new SleepTask(200),
                                new ParallelTask(
                                        new HorizontalSlidesTask(robot, 21, 1.0, 300),
                                        new IntakeArmTask(robot, IntakeArmTask.Position.DOWN)
                                )
                        )
                )
        );
//        task.add(new SleepTask(1000));
        task.add(
                new ParallelTask(
                        new IntakeArmTask(robot, IntakeArmTask.Position.SLIGHTLY_UP),
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose2d pos1 = getSecondSampleOuttakePos();
                                    return builder
                                            .turnTo(pos1.heading, new TurnConstraints(4*Math.PI, -Math.PI*4, Math.PI*4))
                                            .build();
                                }
                        ),
                        new SeriesTask(
                                new ParallelTask(
                                        new UnboundedIntakeTask(robot, 0.5),
                                        new SleepTask(600)
                                ),
                                new ParallelTask(
                                        new UnboundedIntakeTask(robot, -1.0),
                                        new SleepTask(200)
                                )
                        ),
                        new SeriesTask(
                                new HorizontalSlidesTask(robot, 15.0, 1.0, 300),
                                new SleepTask(200),
                                new HorizontalSlidesTask(robot, 19.0, 1.0, 300)
                        )
                )
        );
        task.add(
                new SeriesTask(
                        new ConditionalTask(new SampleStateCondition(robot)),
                        new ParallelTask(
                                new UnboundedIntakeTask(robot, 1.0),
                                new SleepTask(200)
                        ),
                        new IntakeTask(robot,400,-1.0)
                )
        );
//        new SleepTask(1000);
        task.add(
                new ParallelTask(
                        new UnboundedIntakeTask(robot, 1.0),
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose2d pos1 = getThirdSampleIntakePos();
                                    return builder
                                            .strafeToLinearHeading(pos1.position, pos1.heading, new TranslationalVelConstraint(40))
                                            .build();
                                }
                        ),
                        new SeriesTask(
                                new TimedHorizontalSlidesTask(robot, 15, 1.0, 500),
                                new SleepTask(200),
                                new ParallelTask(
                                        new HorizontalSlidesTask(robot, 21, 1.0, 300),
                                        new IntakeArmTask(robot, IntakeArmTask.Position.DOWN)
                                )
                        )
                )
        );
//        task.add(new SleepTask(1000));
        task.add(
                new ParallelTask(
                        new IntakeArmTask(robot, IntakeArmTask.Position.SLIGHTLY_UP),
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose2d pos1 = getThirdSampleOuttakePos();
                                    return builder
                                            .turnTo(pos1.heading, new TurnConstraints(4*Math.PI, -Math.PI*4, Math.PI*4))
                                            .build();
                                }
                        ),
                        new SeriesTask(
                                new ParallelTask(
                                        new UnboundedIntakeTask(robot, 0.5),
                                        new SleepTask(600)
                                ),
                                new ParallelTask(
                                        new UnboundedIntakeTask(robot, -1.0),
                                        new SleepTask(200)
                                )
                        ),
                        new SeriesTask(
                                new HorizontalSlidesTask(robot, 15.0, 1.0, 300),
                                new SleepTask(200),
                                new HorizontalSlidesTask(robot, 19.0, 1.0, 300)
                        )
                )
        );
        task.add(
                new SeriesTask(
                        new ConditionalTask(new SampleStateCondition(robot)),
                        new ParallelTask(
                                new UnboundedIntakeTask(robot, 1.0),
                                new SleepTask(300)
                        ),
                        new IntakeTask(robot,400,-1.0)
                )
        );
//        new SleepTask(1000);
        return task;
    }
    @Override
    protected Task createCycleTask(int cycleNumber) {
        state = AutoState.CYCLE;
        SeriesTask task = new SeriesTask();
        task.add(
                new ParallelTask(
                        new HorizontalSlidesTask(robot, 0, 1.0, 700),
                        new VerticalSlidesTask(robot, 0, 1.0, 1000),
                        new SeriesTask(
                                new SleepTask(1000),
                                new BucketTask(robot, BucketTask.Position.FLAT)
                        ),
                        new SeriesTask(
                                new SleepTask(CLAW_OPEN_DELAY),
                                new ClawTask(robot, ClawTask.ClawPosition.OPEN)
                        ),
                        new SeriesTask(
                                new SleepTask(100),
                                new RuntimeDrivingTask(
                                        robot,
                                        builder -> {
                                            Pose2d pos = getSpecimenIntakePos(cycleNumber);
                                            return builder.strafeToLinearHeading(pos.position, pos.heading).build();
                                        }
                                )
                        )
                )
        );
//        task.add(new SleepTask(500));
        task.add(
                new ParallelTask(
                        new ClawTask(robot, ClawTask.ClawPosition.CLOSE),
                        new ParallelTask(
                                new SeriesTask(
                                        new SleepTask(50),
                                        new VerticalSlidesTask(robot, DELIVERY_CHAMBER_HEIGHT, 1.0, 2000)
                                ),
                                new SeriesTask(
                                    new SleepTask(500),
                                    new BucketTask(robot, BucketTask.Position.CHAMBER_DELIVER)
                                ),
                                new RuntimeDrivingTask(
                                        robot,
                                        builder -> {
                                            Pose2d pos = getSubmersibleDeliverPos(cycleNumber);
                                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(60)).build(); //TODO: possibly 50 or 60
                                        }
                                )
                        )
                )
        );
//        task.add(new SleepTask(500));
        return task;
    }
    @Override
    protected Task createFinishTask() {
        state = AutoState.FINISH;
        SeriesTask task = new SeriesTask();
        task.add(
                new ParallelTask(
                        new SeriesTask(
                                new SleepTask(400),
                                new HorizontalSlidesTask(robot, 22.0, 1.0, 2000)
                        ),
                        new VerticalSlidesTask(robot, 0, 1.0, 1000),
                        new SeriesTask(
                                new SleepTask(1000),
                                new BucketTask(robot, BucketTask.Position.FLAT)
                        ),
                        new SeriesTask(
                                new SleepTask(CLAW_OPEN_DELAY),
                                new ClawTask(robot, ClawTask.ClawPosition.OPEN)
                        ),
                        new SeriesTask(
                                new SleepTask(300),
                                new RuntimeDrivingTask(
                                        robot,
                                        builder -> {
                                            Pose2d pos = getParkPos(); //getParkPos();
                                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(60)).build();
                                        }
                                )
                        )
                )
        );
        return task;
    }

    protected abstract Pose2d getPreloadSubmersibleDeliverPos();
    protected abstract Pose2d getFirstSampleIntakePos();
    protected abstract Pose2d getFirstSampleOuttakePos();

    protected abstract Pose2d getSecondSampleIntakePos();
    protected abstract Pose2d getSecondSampleOuttakePos();
    protected abstract Pose2d getThirdSampleIntakePos();
    protected abstract Pose2d getThirdSampleOuttakePos();

    protected abstract Pose2d getSpecimenIntakePos(int cycleNumber);
    protected abstract Pose2d getSubmersibleDeliverPos(int cycleNumber);
    protected abstract Pose2d getParkPos();
}
