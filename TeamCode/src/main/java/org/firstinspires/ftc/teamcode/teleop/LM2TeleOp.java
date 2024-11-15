package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.V2RobotBase;
import org.firstinspires.ftc.teamcode.task.BucketTask;
import org.firstinspires.ftc.teamcode.task.ClawTask;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.VerticalSlidesTask;
import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "League Meet 2 TeleOp")
@Config

public class LM2TeleOp extends LinearOpMode {

    public static double VERTICAL_STAND_POWER = 0.001;
    public static double VERTICAL_SLIDE_MAX_POWER = 1.0;
    public static double MIN_DRIVE_POWER = 0.01;
    public static double DELIVERY_CHAMBER_HEIGHT = 18.2745;

    private V2RobotBase robot;
    private GamePad gp1, gp2;
    private ElapsedTime cycleTimer;
    private double cycleSum, cycleCnt, cycleAvg;
    private boolean clawOpen;

    private Task horizontalSlidesTask,
            verticalSlidesTask,
            resetTask;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = (V2RobotBase) RobotFactory.createRobot(hardwareMap);
        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);

        robot.teleOpInit();
        waitForStart();

        cycleTimer = new ElapsedTime();

        while (opModeIsActive()) {
            driveRobot();
            runIntake();
            runHorizontalSlides();
            runDelivery();

            if(resetTask==null) checkReset();

            update();
        }
    }
    private void update() {
        if (horizontalSlidesTask != null && horizontalSlidesTask.perform()) {
            horizontalSlidesTask = null;
        } if (verticalSlidesTask != null && verticalSlidesTask.perform()) {
            verticalSlidesTask = null;
        } if (resetTask != null && resetTask.perform()) {
            resetTask = null;
        }

        gp1.update();
        gp2.update();
        updateTelemetry();
        robot.updateEverything();
    }

    private void updateTelemetry() {
        telemetry.addData("colors", robot.getIntakeColorToString());
        telemetry.addData("cycle time", cycleTimer.milliseconds());
        telemetry.addData("average cycle time (seconds)", cycleAvg);
        telemetry.addData("pose", robot.pose);
        telemetry.addData("vertical slide position", robot.getVerticalSlidesHeight());
        telemetry.update();
    }

    private void checkReset() {
        if(gp1.onceRightBumper() || gp2.onceRightBumper()) {
            resetTask = new SeriesTask(
                    new BucketTask(robot, BucketTask.Position.TRANSFER),
                    new ParallelTask(
                            new VerticalSlidesTask(robot, 0, 0.9, 3000),
                            new HorizontalSlidesTask(robot, V2RobotBase.HORIZONTAL_SLIDE_TRANSFER_POS, 0.9,1500),
                            new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER)
                    ),
                    new SleepTask(1000)
            );
        }
    }

    private void runHorizontalSlides() {
        if(gp2.onceLeftBumper()) {
            if (horizontalSlidesTask != null) {
                horizontalSlidesTask.cancel();
                horizontalSlidesTask = null;
            } else {
                horizontalSlidesTask = new HorizontalSlidesTask(robot, V2RobotBase.HORIZONTAL_SLIDE_TRANSFER_POS, 0.9, 2000);
            }
        }
        else if(horizontalSlidesTask==null){
            if(-gp2.leftStickY()>0.3) {
                robot.setHorizontalSlidesPower(0.8);
            }
            else if(-gp2.leftStickY()<-0.3) {
                robot.setHorizontalSlidesPower(-0.8);
            }
            else if(resetTask==null){
                robot.setHorizontalSlideDist(robot.getHorizontalSlideDist(), 0.2);
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
            a = -gp1.rightStickX() * 0.8;
        }
        double pow = Math.sqrt(x * x + y * y);
//        double limPow = driveLim.calculate(pow);
//        if (pow > MIN_DRIVE_POW) {
//            x *= limPow / pow;
//            y *= limPow / pow;
//        }
//        a = turnLim.calculate(a);
//        telemetry.addData("driving", x + " " + y + " " + a);
        robot.setDrivePowers(new PoseVelocity2d(new Vector2d(x, y), a));
    }


    private void runDelivery() {

        // Slides
        if(gp2.dpadUp() || gp2.dpadDown()){
            if(verticalSlidesTask != null) {
                verticalSlidesTask.cancel();
                verticalSlidesTask = null;
            }
            robot.setVerticalSlidePowerWithoutEncoder(gp2.dpadDown()? -0.9 : 1.0);
        } else if(resetTask == null && verticalSlidesTask == null) {
            robot.setVerticalSlidePowerWithoutEncoder(robot.getVerticalSlidesHeight() < 0.5? 0:VERTICAL_STAND_POWER);
        }

        // Bucket
        if(gp1.onceX()) {
            robot.setBucketServoPosition(BucketTask.Position.FLAT);
        } else if(!gp1.back() && gp1.onceA()) {
            robot.setBucketServoPosition(BucketTask.Position.TRANSFER);
        } else if(gp1.onceY()) {
            robot.setBucketServoPosition(BucketTask.Position.DELIVER);
            if(robot.getVerticalSlidesHeight() >= 12) {
                cycleSum += cycleTimer.seconds(); cycleCnt++; cycleAvg = cycleSum/cycleCnt;
                cycleTimer.reset();
            }
        } else if (gp1.onceB()) {
            robot.setBucketServoPosition(BucketTask.Position.CHAMBER_DELIVER);
            if(verticalSlidesTask == null) {
                verticalSlidesTask = new VerticalSlidesTask(robot, DELIVERY_CHAMBER_HEIGHT, 1.0, 3000);
            }
        }

        // Claw
        if(gp2.onceB()){
            if(clawOpen) {
                robot.setClawPosition(ClawTask.ClawPosition.CLOSE);
            } else {
                robot.setClawPosition(ClawTask.ClawPosition.OPEN);
                robot.setBucketServoPosition(BucketTask.Position.FLAT);
            }
            clawOpen = !clawOpen;
        }

        // Rung Stick
        if(gp1.back() && gp1.onceA()){
            robot.toggleRungStick();
        }
    }

    private void runIntake() {

        if(gp2.onceY()) {
            robot.setIntakeArmPos(IntakeArmTask.Position.TRANSFER);
        } else if(gp2.onceX()) {
            robot.setIntakeArmPos(IntakeArmTask.Position.CLEAR);
        } else if(gp2.onceA()) {
            robot.setIntakeArmPos(IntakeArmTask.Position.DOWN);
        }

        if(gp2.rightTrigger()>0.1){
            robot.runIntakeIn();
        } else if(gp2.leftTrigger()>0.1) {
            robot.runIntakeOut();
        } else{
            robot.stopIntake();
        }
    }
}
