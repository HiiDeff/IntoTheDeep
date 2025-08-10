package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.SensorUpdateThread;
import org.firstinspires.ftc.teamcode.drive.V308Robot.V308Robot;
import org.firstinspires.ftc.teamcode.task.ClawTask;
import org.firstinspires.ftc.teamcode.task.DeliveryArmTask;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.IntakeClawTask;
import org.firstinspires.ftc.teamcode.task.IntakePivotTask;
import org.firstinspires.ftc.teamcode.task.IntakeWristTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.SwipeStickTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.TimedHorizontalSlidesTask;
//import org.firstinspires.ftc.teamcode.task.UnboundedIntakeTask;
import org.firstinspires.ftc.teamcode.task.TurretTask;
import org.firstinspires.ftc.teamcode.task.VerticalSlidesTask;
import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "V308 (Andrew Testing) TeleOp")
@Config
public class V308RobotTeleOp extends LinearOpMode {

    public static int PRESET_CYCLE_CNT = 3, CLAW_OPEN_DELAY = 420;

    public static double VERTICAL_STAND_POWER = 0.2;
    public static double VERTICAL_SLIDE_MAX_POWER = 1.0;
    public static double MIN_DRIVE_POWER = 0.01;
    public static double DELIVERY_CHAMBER_HEIGHT = 21;
    public static double DELIVERY_HIGH_BASKET_HEIGHT = 38.6;
    public static double DELIVERY_LOW_BASKET_HEIGHT = 24.0;
    public static double MAX_SLIDE_EXTENSION_INCH = 21.0, MAX_SLIDE_EXTENSION_INCH_WITH_HANG = 17.5;

    public static double CYCLE_DELIVER_X = 23, CYCLE_DELIVER_Y = 40, Y_OFFSET = 1.5;

    public MultipleTelemetry multipleTelemetry;
    public ElapsedTime elapsedTime = new ElapsedTime();

    private V308Robot robot;
    private GamePad gp1, gp2;
    private ElapsedTime cycleTimer;
    private double cycleSum, cycleCnt, cycleAvg;
    private boolean clawOpen, horizontalPositionUpdated, hangUp, swipeStickOut;
    private Pose2d deliveryPose;

    private Task submersibleTask,
            transferTask,
            deliverChamberTask,
            deliverChamberTaskLite,
            deliverTask,
            swipeStickTask,
            intakeTask;
    private SensorUpdateThread sensorUpdateThread;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = (V308Robot) RobotFactory.createRobot(hardwareMap);
        robot.teleOpInit();
        SwipeStickTask.setPosition(robot, SwipeStickTask.Position.IN);
        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);
//        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        cycleTimer = new ElapsedTime();

        sensorUpdateThread = new SensorUpdateThread(robot);
        sensorUpdateThread.start();


        while (opModeIsActive()) {

            driveRobot();
            if (deliverChamberTask == null && deliverChamberTaskLite == null) runIntake();
            if (deliverChamberTask == null && deliverChamberTaskLite == null) runHorizontalSlides();
            if (deliverChamberTask == null && deliverChamberTaskLite == null) runDelivery();
            checkPreloads();
            update();
//            Log.i("ethan ", ""+ elapsedTime.milliseconds());
            elapsedTime.reset();
        }
        sensorUpdateThread.interrupt();
    }

    private void update() {
        if (submersibleTask != null && submersibleTask.perform()) submersibleTask = null;
        if (transferTask != null && transferTask.perform()) transferTask = null;
        if (deliverChamberTask != null && deliverChamberTask.perform()) deliverChamberTask = null;
        if (deliverChamberTaskLite != null && deliverChamberTaskLite.perform()) deliverChamberTaskLite = null;
        if (deliverTask != null && deliverTask.perform()) deliverTask = null;
        if (swipeStickTask != null && swipeStickTask.perform()) swipeStickTask = null;
        if (intakeTask != null && intakeTask.perform()) intakeTask = null;
        gp1.update();
        gp2.update();
        updateTelemetry();
        robot.updateEverything();
    }

    private void updateTelemetry() {
//        telemetry.addData("colors", robot.getIntakeColorToString());
//        telemetry.addData("cycle time", cycleTimer.milliseconds());
//        telemetry.addData("average cycle time (seconds)", cycleAvg);
//        telemetry.addData("pose", robot.pose);
//        telemetry.addData("vertical slide position", robot.getVerticalSlidesHeightInch());
//        telemetry.addData("horizontal slide position", robot.getHorizontalSlideDist());
//        telemetry.update();

//        multipleTelemetry.addData("current bucket servo position", robot.bucketProfiler.getCurPos());
//        multipleTelemetry.addData("target bucket servo position", robot.bucketProfiler.getTargetPos());
//        multipleTelemetry.update();
        telemetry.addData("isSampleIn", robot.isSampleIn());
        telemetry.update();
    }

    private void checkPreloads() {
        if (submersibleTask != null || transferTask != null || deliverChamberTaskLite != null || deliverChamberTask != null || swipeStickTask != null)
            return;
        // Submersible Intake Preset
        if (gp2.back() && gp2.onceY()) {
            submersibleTask =  new SeriesTask(
                    new ParallelTask(
                            new IntakeArmTask(robot, IntakeArmTask.Position.CLEAR),
                            new HorizontalSlidesTask(robot, 9.0, 1.0, 500)
                    ),
                    new IntakeArmTask(robot, IntakeArmTask.Position.DOWN)
            );
        }
        // Transfer Preset
        else if (gp2.onceX()) {
            transferTask = new SeriesTask(
                    new ParallelTask(
                            new DeliveryArmTask(robot, DeliveryArmTask.Position.TRANSFER),
                            new ClawTask(robot, ClawTask.ClawPosition.OPEN)
                    ),
                    new ParallelTask(
                            new VerticalSlidesTask(robot, 0, 1.0, 2000),
                            new SeriesTask(
                                    new ParallelTask(
                                            new TimedHorizontalSlidesTask(robot, robot.getHorizontalSlideTransferPos(), 1.0,1000), //TODO: increase this
                                            new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER)
                                            //new UnboundedIntakeTask(robot, 1.0)
                                    ),
                                    new ParallelTask(
                                            new TimedHorizontalSlidesTask(robot, robot.getHorizontalSlideTransferPos(), 1.0, 500)
                                            //new UnboundedIntakeTask(robot, -1.0)
                                    ),
                                    new ParallelTask(
                                            new IntakeArmTask(robot, IntakeArmTask.Position.CLEAR),
                                            new DeliveryArmTask(robot, DeliveryArmTask.Position.CARRY)
                                    )
                            )
                    )
            ); clawOpen = true;
        }
        // Chamber Preset
        else if (gp1.onceLeftTrigger()) {
            createSpecimenDeliveryTask(); clawOpen = true;
//            deliverChamberTask = new SeriesTask(
//                    new ClawTask(robot, ClawTask.ClawPosition.CLOSE),
//                    new ParallelTask(
//                            new VerticalSlidesTask(robot, DELIVERY_CHAMBER_HEIGHT, 1.0, 700),
//                            new BucketTask(robot, BucketTask.Position.CHAMBER_DELIVER)
//                    )
//            );
//            clawOpen = false;
        } else if (gp1.onceX()) {
            createSpecimenDeliveryTaskLite(); clawOpen = true;
        }
        // Deliver Preset
        else if (deliverTask == null && gp2.onceB()) {
            double height = gp2.back() ? DELIVERY_LOW_BASKET_HEIGHT : DELIVERY_HIGH_BASKET_HEIGHT;
            deliverTask =
                    new ParallelTask(
                            new VerticalSlidesTask(robot, height, 1.0, 1500),
                            new DeliveryArmTask(robot, DeliveryArmTask.Position.CARRY)
                    );
        }
        else if(deliverTask == null && gp2.onceY()){
            deliverTask = new ParallelTask(
                    new DeliveryArmTask(robot, DeliveryArmTask.Position.FLAT),
                    new ClawTask(robot, ClawTask.ClawPosition.CLOSE),
                    new VerticalSlidesTask(robot, 14, 1.0, 500)
            );
            clawOpen = false;
        }
    }

    private void runHorizontalSlides() { //TODO: check this logic
        if ((-gp2.leftStickY() > 0.3 && robot.getHorizontalSlideDistInch() < (robot.isHangUp()? MAX_SLIDE_EXTENSION_INCH_WITH_HANG : MAX_SLIDE_EXTENSION_INCH)) || -gp2.leftStickY() < -0.3) {
            if (submersibleTask != null) {
                submersibleTask.cancel();
                submersibleTask = null;
            }
            if (transferTask != null) {
                transferTask.cancel();
                transferTask = null;
            }
            robot.setHorizontalSlidesPowerWithoutEncoder(-gp2.leftStickY() > 0.3 ? 0.8 : -0.8);
            if (gp2.rightBumper()) {
                robot.resetHorizontalAndVerticalSlidesEncoderPositions();
            } else {
                robot.updateLastHorizontalEncoder();
                horizontalPositionUpdated = true;
            }
        } else if (submersibleTask == null && transferTask == null) {
            if (horizontalPositionUpdated) {
                robot.brakeHorizontalSlides(0.05); // TODO: CHECK THIS
                horizontalPositionUpdated = false;
            }
        }
    }

    private void driveRobot() {
        double x = 0, y = 0, a = 0;
        if (gp1.dpadLeft() || gp1.dpadRight()) {
            a = 0.5 * (gp1.dpadLeft() ? 1 : -1);
        } else if (gp1.dpadUp() || gp1.dpadDown()) {
            x = 0.3 * (gp1.dpadUp() ? 1 : -1);
        } else {
            x = -gp1.leftStickY();
            y = -gp1.leftStickX();
            a = -gp1.rightStickX() * 0.9;
        }
        double pow = Math.sqrt(x * x + y * y);
//        double limPow = driveLim.calculate(pow);
//        if (pow > MIN_DRIVE_POW) {
//            x *= limPow / pow;
//            y *= limPow / pow;
//        }
//        a = turnLim.calculate(a);
//        telemetry.addData("driving", x + " " + y + " " + a);
        if (deliverChamberTask != null) {
            if (pow > 0.1) {
                deliverChamberTask.cancel();
                deliverChamberTask = null;
            }
        } else {
            robot.setDrivePowers(new PoseVelocity2d(new Vector2d(x, y), a));
        }
    }

    private void runDelivery() {
        // Slides
        if ((gp2.dpadUp() || gp2.dpadDown())) {
            if (deliverChamberTask != null) {
                deliverChamberTask.cancel();
                deliverChamberTask = null;
            }
            else if(deliverChamberTaskLite != null) {
                deliverChamberTaskLite.cancel();
                deliverChamberTaskLite = null;
            }
            else if (deliverTask != null) {
                deliverTask.cancel();
                deliverTask = null;
            } else if (transferTask != null) {
                transferTask.cancel();
                transferTask = null;
            }
            if (gp2.dpadUp()) {
                robot.setVerticalSlidePowerWithoutEncoder(1.0);
            } else {
                robot.setVerticalSlidePowerWithoutEncoder(-0.5);
            }
        } else if (deliverChamberTaskLite == null && deliverChamberTask == null && deliverTask == null && transferTask == null) {
            robot.setVerticalSlidePowerWithoutEncoder(robot.getVerticalSlidesHeightInch() < 1.0 ? 0 : VERTICAL_STAND_POWER);
        }
        // Bucket
        if (gp1.onceB()) {
            robot.setDeliveryArmServoPosition(DeliveryArmTask.Position.FLAT);
            robot.setClawPosition(ClawTask.ClawPosition.OPEN); clawOpen = true;
        } else if (!gp1.back() && gp1.onceY()) {
            robot.setDeliveryArmServoPosition(DeliveryArmTask.Position.DELIVER);
            robot.setClawPosition(ClawTask.ClawPosition.OPEN); clawOpen = true;
            if (robot.getVerticalSlidesHeightInch() >= 12) {
                cycleSum += cycleTimer.seconds(); cycleCnt++; cycleAvg = cycleSum / cycleCnt;
                cycleTimer.reset();
            }
        } else if (!gp1.back() && !gp1.start() && gp1.onceA()) {
            robot.setDeliveryArmServoPosition(DeliveryArmTask.Position.TRANSFER);
            robot.setClawPosition(ClawTask.ClawPosition.OPEN); clawOpen = true;
        }
        // Claw
        if (gp2.onceA()) {
            robot.setClawPosition(clawOpen ? ClawTask.ClawPosition.CLOSE : ClawTask.ClawPosition.OPEN);
            clawOpen = !clawOpen;
        }
        // Hang
        if (gp1.leftBumper() && gp1.onceRightBumper() && robot.getHorizontalSlideDistInch()<MAX_SLIDE_EXTENSION_INCH_WITH_HANG) {
            if (!hangUp) {
                robot.releaseHang();
            } else {
                robot.lowerHang();
                robot.setDeliveryArmServoPosition(DeliveryArmTask.Position.FLAT);
            }
            hangUp = !hangUp;
        }
        if(gp1.back() && gp1.onceY()) {
            robot.initHang();
        }
        //Swipe Stick
        if (gp1.onceRightTrigger()) {
            if (!swipeStickOut) {
                SwipeStickTask.setPosition(robot, SwipeStickTask.Position.OUT);
            } else {
                SwipeStickTask.setPosition(robot, SwipeStickTask.Position.IN);
            }
            swipeStickOut = !swipeStickOut;
        }
    }

    public static double INTAKE_PIVOT_CENTER_ANGLE = 90;

    private void runIntake() {
        //if (gp2.onceRightTrigger()){

        //}

        if (gp2.onceRightTrigger()) {
            intakeTask = new IntakePivotTask(robot, INTAKE_PIVOT_CENTER_ANGLE);
        }
        if (intakeTask == null){
            intakeTask = new ParallelTask(
                    new IntakeWristTask(robot, IntakeWristTask.WristPosition.DOWN),
                    new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                    new TurretTask(robot, TurretTask.TurretPosition.CENTER),
                    new IntakeClawTask(robot, IntakeClawTask.ClawPosition.OPEN)
                    );
        }

//        if (-gp2.rightStickY() > 0.5) {
//            robot.setIntakeArmPos(IntakeArmTask.Position.TRANSFER);
//        } else if (gp2.onceRightStickButton()) {
//            robot.setIntakeArmPos(IntakeArmTask.Position.CLEAR);
//        } else if (-gp2.rightStickY() < -0.5) {
//            robot.setIntakeArmPos(IntakeArmTask.Position.DOWN);
//        }
//        if (gp2.rightTrigger() > 0.1){
//            robot.runIntakeIn();
//        } else if (gp2.leftTrigger() > 0.1) {
//            robot.runIntakeOut();
//        } else {
//            robot.stopIntake();
//        }

    }

    private void createSpecimenDeliveryTaskLite() {
        if (robot.getVerticalSlidesHeightInch() > 15) {
            deliverChamberTaskLite = new ParallelTask(
                    new HorizontalSlidesTask(robot, 0, 1.0, 700),
                    new VerticalSlidesTask(robot, 0, 1.0, 1000),
                    new SeriesTask(
                            new SleepTask(1000),
                            new DeliveryArmTask(robot, DeliveryArmTask.Position.FLAT)
                    ),
                    new SeriesTask(
                            new SleepTask(CLAW_OPEN_DELAY),
                            new ClawTask(robot, ClawTask.ClawPosition.OPEN)
                    )
            );
        } else {
            deliverChamberTaskLite = new SeriesTask(
                    new ClawTask(robot, ClawTask.ClawPosition.CLOSE),
                    new ParallelTask(
                            new SeriesTask(
                                    new SleepTask(50),
                                    new VerticalSlidesTask(robot, DELIVERY_CHAMBER_HEIGHT, 1.0, 2000)
                            ),
                            new SeriesTask(
                                    new SleepTask(500),
                                    new DeliveryArmTask(robot, DeliveryArmTask.Position.CHAMBER_DELIVER)
                            )
                    )
            );
        }
    }

    private void createSpecimenDeliveryTask() {
        if (deliveryPose == null) {
            //set pos to 0
        }
        SeriesTask task = new SeriesTask();
        if (robot.getVerticalSlidesHeightInch() > 15) {
            if (deliveryPose != null) {
                deliveryPose = new Pose2d(CYCLE_DELIVER_X,
                        Y_OFFSET,
                        Math.PI - 0.05);
            }
            task.add(
                    new ParallelTask(
                            new HorizontalSlidesTask(robot, 0, 1.0, 700),
                            new VerticalSlidesTask(robot, 0, 1.0, 1000),
                            new SeriesTask(
                                    new SleepTask(1000),
                                    new DeliveryArmTask(robot, DeliveryArmTask.Position.FLAT)
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
                                                Pose2d pos = new Pose2d(-1, 0, 0.15);
                                                return builder.strafeToLinearHeading(pos.position, pos.heading).build();
                                            }
                                    )
                            )
                    )
            );
        }
        if (deliveryPose == null) {
            deliveryPose = new Pose2d(CYCLE_DELIVER_X, CYCLE_DELIVER_Y, Math.PI - 0.05);
        }
        task.add(
                new ParallelTask(
                        new ClawTask(robot, ClawTask.ClawPosition.CLOSE),
                        new SeriesTask(
                                new SleepTask(300),
                                new VerticalSlidesTask(robot, DELIVERY_CHAMBER_HEIGHT, 1.0, 2000)
                        ),
                        new SeriesTask(
                                new SleepTask(700),
                                new DeliveryArmTask(robot, DeliveryArmTask.Position.CHAMBER_DELIVER)
                        ),
                        new SeriesTask(
                                new SleepTask(200),
                                new RuntimeDrivingTask(
                                        robot,
                                        builder -> builder.strafeToLinearHeading(deliveryPose.position, deliveryPose.heading, new TranslationalVelConstraint(60)).build()
                                )
                        )
                )
        );
        deliverChamberTask = task;
    }
}
