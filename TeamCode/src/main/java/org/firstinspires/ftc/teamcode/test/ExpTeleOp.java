package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.common.CommandManager;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.V4Robot.V4Robot;
import org.firstinspires.ftc.teamcode.task.ClawTask;
import org.firstinspires.ftc.teamcode.task.DeliveryArmTask;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.RunTillTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.SmartIntakeTask;
import org.firstinspires.ftc.teamcode.task.SwipeStickTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.VerticalSlidesTask;
import org.firstinspires.ftc.teamcode.util.GamePad;
import org.firstinspires.ftc.teamcode.util.SampleColor;

@Config
public abstract class ExpTeleOp extends LinearOpMode {

    // Delays in millis.
    public static int DELAY_DELIVERY_ARM_AFTER_DELIVERY = 200;
    // Delays in slide distance (inch).
    public static double DELAY_DELIVERY_ARM_BEFORE_DELIVERY = 6.0;

    public static double INTAKE_POWER = 1.0;
    public static double HORIZONTAL_SLIDE_POWER = 0.8;
    public static double DELIVERY_HIGH_BASKET_HEIGHT = 28.0;
    public static double DELIVERY_LOW_BASKET_HEIGHT = 12.0;
    public static double MAX_SLIDE_EXTENSION_INCH = 21.0;
    public MultipleTelemetry multipleTelemetry;
    public ElapsedTime elapsedTime = new ElapsedTime();
    private V4Robot robot;
    private GamePad gp1, gp2;
    private boolean horizontalPositionUpdated = false;

    private Task currTask = null;

    private CommandManager commandManager;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = (V4Robot) RobotFactory.createRobot(hardwareMap);
        robot.teleOpInit();
        SwipeStickTask.setPosition(robot, SwipeStickTask.Position.IN);
        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);
        commandManager = new CommandManager(gp1, gp2);
        waitForStart();

        while (opModeIsActive()) {
            driveRobot();
            cancelTaskIfNeeded();
            if (!isTaskRunning()) {
                switch (commandManager.getCommand()) {
                    case INTAKE_START:
                        startIntakeTask();
                        break;
                    case TRANSFER:
                        startTransferTask();
                        break;
                    case TO_BUCKET_LOW:
                        startToLowBucketTask();
                        break;
                    case TO_BUCKET_HIGH:
                        startToHighBucketTask();
                        break;
                    case DELIVERY_BUCKET:
                        startBucketDeliveryTask();
                        break;
                    default:
                        break;
                }
            }
            runHorizontalSlides();
            update();
            elapsedTime.reset();
        }
    }

    protected abstract SampleColor getTargetColor();

    private void update() {
        commandManager.update();
        gp1.update();
        gp2.update();
        updateTelemetry();
        robot.updateEverything();
    }

    private void updateTelemetry() {
        telemetry.addData("colors", robot.getIntakeColorToString());
        telemetry.addData("Loop time", (int) elapsedTime.milliseconds());
//        telemetry.addData("cycle time", cycleTimer.milliseconds());
//        telemetry.addData("average cycle time (seconds)", cycleAvg);
//        telemetry.addData("pose", robot.pose);
        telemetry.addData("vertical slide position", robot.getVerticalSlidesHeightInch());
        telemetry.addData("horizontal slide position", robot.getHorizontalSlideDistInch());
        telemetry.update();

//        multipleTelemetry.addData("current bucket servo position", robot.bucketProfiler.getCurPos());
//        multipleTelemetry.addData("target bucket servo position", robot.bucketProfiler.getTargetPos());
//        multipleTelemetry.update();
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
//        if (deliverChamberTask != null) {
//            if (pow > 0.1) {
//                deliverChamberTask.cancel();
//                deliverChamberTask = null;
//            }
//        } else {
        robot.setDrivePowers(new PoseVelocity2d(new Vector2d(x, y), a));
//        }
    }

    private void runHorizontalSlides() {
        if (robot.getHorizontalSlideDistInch() < MAX_SLIDE_EXTENSION_INCH &&
                Math.abs(gp2.leftStickY()) > 0.3) {
            robot.setHorizontalSlidesPowerWithoutEncoder(-gp2.leftStickY() > 0.3 ?
                    HORIZONTAL_SLIDE_POWER : -HORIZONTAL_SLIDE_POWER);
            if (gp2.rightBumper()) {
                robot.resetHorizontalAndVerticalSlidesEncoderPositions();
            } else {
                robot.updateLastHorizontalEncoder();
                horizontalPositionUpdated = true;
            }
        } else if (horizontalPositionUpdated) {
            robot.brakeHorizontalSlides(0.05); // TODO: CHECK THIS
            horizontalPositionUpdated = false;
        }

    }

    private void startIntakeTask() {
        currTask = new SmartIntakeTask(robot, getTargetColor(), INTAKE_POWER);
    }

    private void startTransferTask() {
        SeriesTask task = new SeriesTask();
        task.add(new ParallelTask(
                new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER),
                new DeliveryArmTask(robot, DeliveryArmTask.Position.TRANSFER),
                new HorizontalSlidesTask(robot, 0, 1.0, 500),
                new VerticalSlidesTask(robot, 0),
                new ClawTask(robot, ClawTask.ClawPosition.OPEN)
        ));
        task.add(new ClawTask(robot, ClawTask.ClawPosition.CLOSE));
        task.add(new ParallelTask(
                new IntakeArmTask(robot, IntakeArmTask.Position.UP),
                new DeliveryArmTask(robot, DeliveryArmTask.Position.UP)
        ));
        currTask = task;
    }

    private void startToLowBucketTask() {
        startToBucketTask(DELIVERY_LOW_BASKET_HEIGHT);
    }


    private void startToHighBucketTask() {
        startToBucketTask(DELIVERY_HIGH_BASKET_HEIGHT);
    }

    private void startToBucketTask(double height) {
        currTask = new ParallelTask(
                new VerticalSlidesTask(robot, height),
                new SeriesTask(
                        new RunTillTask(() ->
                                robot.getVerticalSlidesHeightInch() >
                                        height - DELAY_DELIVERY_ARM_BEFORE_DELIVERY),
                        new DeliveryArmTask(robot, DeliveryArmTask.Position.DELIVER)
                )
        );
    }

    private void startBucketDeliveryTask() {
        currTask = new SeriesTask(
                new ClawTask(robot, ClawTask.ClawPosition.OPEN),
                new ParallelTask(
                        new DeliveryArmTask(robot, DeliveryArmTask.Position.TRANSFER),
                        new SeriesTask(
                                new SleepTask(DELAY_DELIVERY_ARM_AFTER_DELIVERY),
                                new VerticalSlidesTask(robot, 0)
                        )
                )
        );
    }

    private void cancelTaskIfNeeded() {
        if (currTask != null && commandManager.getCommand() == CommandManager.Command.CANCEL) {
            RobotLog.i("lswu-debug teleop", "canceling tasks");
            currTask.cancel();
            currTask = null;
        }
    }

    private boolean isTaskRunning() {
        boolean isRunning = currTask != null;

        if (isRunning && currTask.perform()) {
            currTask = null;
            isRunning = false;
        }
        return isRunning;
    }
}
