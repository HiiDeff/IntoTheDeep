//package org.firstinspires.ftc.teamcode.auto;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.Vector2d;
//
//import org.firstinspires.ftc.teamcode.task.DiffyIntakeTask;
//import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
//import org.firstinspires.ftc.teamcode.task.IntakeReverseTask;
//import org.firstinspires.ftc.teamcode.task.IntakeTask;
//import org.firstinspires.ftc.teamcode.task.ParallelTask;
//import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
//import org.firstinspires.ftc.teamcode.task.SeriesTask;
//import org.firstinspires.ftc.teamcode.task.SleepTask;
//import org.firstinspires.ftc.teamcode.task.Task;
//import org.firstinspires.ftc.teamcode.task.VerticalSlideLeftTask;
//import org.firstinspires.ftc.teamcode.task.VerticalSlideRightTask;
//import org.firstinspires.ftc.teamcode.task.VerticalSlideTask;
//import org.opencv.core.Mat;
//
//public abstract class CloseAuto extends AutoBase{
//    @Override
//    protected Task createStartTask() {
//        SeriesTask task = new SeriesTask();
//        task.add(new HorizontalSlidesTask(robot, 0));
//        task.add(new SeriesTask(
//                new DiffyIntakeTask(robot, DiffyIntakeTask.DiffyPosition.UP),
//                        new ParallelTask(
//                                new RuntimeDrivingTask(
//                                        robot,
//                                        builder -> {
//                                            builder.setTangent(0);
//                                            Pose2d pos = new Pose2d(new Vector2d(7.5, 20.5), Math.PI*3/4);
//                                            return builder.strafeToLinearHeading(pos.position, pos.heading).build();
//                                        }),
//                                new VerticalSlideTask(robot, 54)
//                        )
//        )
//        );
//        task.add(new SeriesTask(
//                //new SleepTask(100),
//                new IntakeReverseTask(robot, 200),
//                new SleepTask(100)
//        ));
//
//        task.add(new SeriesTask(
//                new RuntimeDrivingTask(robot,
//                        builder -> {
//                            Pose2d pos = new Pose2d(new Vector2d(8, 20), Math.PI*3/4);
//                            return builder.strafeToLinearHeading(pos.position, pos.heading).build();
//                        }),
//                new ParallelTask(
//                        new VerticalSlideTask(robot,0),
//                        new DiffyIntakeTask(robot, DiffyIntakeTask.DiffyPosition.HORIZONTAL)
//                ),
//                        new ParallelTask(
//                                new VerticalSlideLeftTask(robot, 5),
//                                new VerticalSlideRightTask(robot, 5)
//                        )
//        )
//        );
//
//        return task;
//    }
//
//    @Override
//    protected Task createCycleTask() {
//        state = AutoState.CYCLE;
//        SeriesTask task = new SeriesTask();
//        task.add(new HorizontalSlidesTask(robot, 0));
//        double intakePosX;
//        double intakePosY;
//
//        if(autoStates.getCycleNumber() == 1){
//            intakePosX = 36;
//            intakePosY = 10;
//        }
//        else if(autoStates.getCycleNumber() == 2){
//            intakePosX = 36;
//            intakePosY = 20;
//        }
//        else if(autoStates.getCycleNumber() == 3){
//            intakePosX = 36;
//            intakePosY = 27;
//        } else {
//            intakePosY = 0;
//            intakePosX = 0;
//        }
//
//        task.add(new SeriesTask(
//                new ParallelTask(
//                        new RuntimeDrivingTask(robot,
//                                builder -> {
//                                    Pose2d pos = new Pose2d(new Vector2d(intakePosX-6, intakePosY-8), Math.PI/2);
//                                    return builder.strafeToLinearHeading(pos.position, pos.heading).build();
//                                }
//                        ),
//                        new ParallelTask(
//                                new VerticalSlideRightTask(robot, 5),
//                                new VerticalSlideLeftTask(robot, 5)
//                        ),
//                        new DiffyIntakeTask(robot, DiffyIntakeTask.DiffyPosition.HORIZONTAL)
//                ),
//
//                new RuntimeDrivingTask(robot,
//                                builder -> {
//                                    Pose2d pos = new Pose2d(new Vector2d(intakePosX, intakePosY), Math.PI/2);
//                                    return builder.strafeToConstantHeading(pos.position).build();
//                }
//                ),
//                new IntakeTask(robot, 50),
//                new VerticalSlideTask(robot, 0),
//                new SleepTask(150),
//                new ParallelTask(
//                        new VerticalSlideRightTask(robot, 5),
//                        new VerticalSlideLeftTask(robot, 5)
//                ),
//                new DiffyIntakeTask(robot, DiffyIntakeTask.DiffyPosition.DOWN_CENTER)
//        ));
//
//        task.add(new SeriesTask(
//                new ParallelTask(
//                        new DiffyIntakeTask(robot, DiffyIntakeTask.DiffyPosition.UP),
//                        new RuntimeDrivingTask(robot,
//                                builder -> {
//                                    Pose2d pos = new Pose2d(new Vector2d(8, 20), Math.PI*3/4);
//                                    return builder.strafeToLinearHeading(pos.position, pos.heading).build();
//                                }
//                                ),
//                        new VerticalSlideTask(robot, 54)
//                )
//        ));
//        task.add(new SeriesTask(
//                new IntakeReverseTask(robot, 200),
//                new SleepTask(100)
//        ));
//        task.add(new SeriesTask(
//                new RuntimeDrivingTask(robot,
//                        builder -> {
//                            Pose2d pos = new Pose2d(new Vector2d(9, 19), Math.PI*3/4);
//                            return builder.strafeToLinearHeading(pos.position, pos.heading).build();
//                        }),
//                new ParallelTask(
//                        new VerticalSlideTask(robot,0),
//                        new DiffyIntakeTask(robot, DiffyIntakeTask.DiffyPosition.HORIZONTAL)
//                ),
//                new ParallelTask(
//                        new VerticalSlideLeftTask(robot, 5),
//                        new VerticalSlideRightTask(robot, 5)
//                )
//        ));
//        task.add(new HorizontalSlidesTask(robot, 5));
//
//        return task;
//    }
//
//    @Override
//    protected Task createFinishTask() {
//        return null;
//    }
//
//
//
//    @Override
//    protected boolean isFar() {
//        return false;
//    }
//}