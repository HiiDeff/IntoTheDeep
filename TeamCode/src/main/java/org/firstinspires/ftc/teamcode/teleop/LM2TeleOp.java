package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.teamcode.task.UnboundedIntakeTask;
import org.firstinspires.ftc.teamcode.task.VerticalSlidesTask;
import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "League Meet 2 TeleOp")
@Config

public class LM2TeleOp extends LinearOpMode {

    public static double VERTICAL_STAND_POWER = 0.01;
    public static double VERTICAL_SLIDE_MAX_POWER = 1.0;
    public static double MIN_DRIVE_POWER = 0.01;
    public static double DELIVERY_CHAMBER_HEIGHT = 20.5;

    private V2RobotBase robot;
    private GamePad gp1, gp2;
    private ElapsedTime cycleTimer;
    private double cycleSum, cycleCnt, cycleAvg;
    private boolean clawOpen, horizontalPositionUpdated;

    private Task submersibleTask,
            transferTask,
            deliverRaiseTask,
            deliverTask;
    ;

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

            checkPreloads();
            update();
        }
    }
    private void update() {
        if (submersibleTask != null && submersibleTask.perform()) {
            submersibleTask = null;
        }if (transferTask != null && transferTask.perform()) {
            transferTask = null;
        }if (deliverRaiseTask != null && deliverRaiseTask.perform()) {
            deliverRaiseTask = null;
        }if (deliverTask != null && deliverTask.perform()) {
            deliverTask = null;
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
        telemetry.addData("horizontal slide position", robot.getHorizontalSlideDist());
        telemetry.update();
    }

    private void checkPreloads() {

        // Submersible Intake Preset
        if(submersibleTask == null && gp2.onceY()){
            submersibleTask = new SeriesTask(
                    new IntakeArmTask(robot, IntakeArmTask.Position.CLEAR),
                    new ParallelTask(
                            new UnboundedIntakeTask(robot),
                            new HorizontalSlidesTask(robot, 13.0, 1.0, 500)
                    ),
                    new ParallelTask(
                            new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                            new HorizontalSlidesTask(robot, 9.0, 1.0, 500)
                    )
            );
        }

        // Transfer Preset
        else if(transferTask == null && gp2.onceX()) {
            transferTask = new SeriesTask(
                    new BucketTask(robot, BucketTask.Position.TRANSFER),
                    new ParallelTask(
                            new VerticalSlidesTask(robot, 0, 1.0, 2000),
                            new HorizontalSlidesTask(robot, V2RobotBase.HORIZONTAL_SLIDE_TRANSFER_POS, 1.0,1500),
                            new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER)
                    ),
                    new SleepTask(1000)
            );
        }

        // Deliver Raise Preset
        else if(deliverRaiseTask == null && gp2.onceA()) {
            deliverRaiseTask = new ParallelTask(
                    new VerticalSlidesTask(robot, 40.0, 1.0, 3000),
                    new BucketTask(robot, BucketTask.Position.CARRY)
            );
        }

        // Deliver Preset
        else if(deliverTask == null && (gp2.onceB())) {
            deliverTask = new VerticalSlidesTask(robot, 0, 1.0, 1500);

//                    new SeriesTask(
//                    new BucketTask(robot, BucketTask.Position.DELIVER),
//                    new SleepTask(1000),
//                    new ParallelTask(
//                            new BucketTask(robot, BucketTask.Position.TRANSFER),
//                            new SeriesTask(
//                                    new SleepTask(300),
//                                    new VerticalSlidesTask(robot, 0, 1.0, 1500)
//                            )
//                    )
//            );
            if(robot.getVerticalSlidesHeight() >= 12) {
                cycleSum += cycleTimer.seconds(); cycleCnt++; cycleAvg = cycleSum/cycleCnt;
                cycleTimer.reset();
            }
        }
    }

    private void runHorizontalSlides() {

        if(submersibleTask==null && transferTask==null){
            if(-gp2.leftStickY()>0.3) {
                robot.setHorizontalSlidesPower(0.8);
                robot.updateHorizontalEncoder();
                horizontalPositionUpdated = true;
            }
            else if(-gp2.leftStickY()<-0.3) {
                if(gp2.rightBumper()){
                    robot.horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.horizontalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                } else{
                    robot.updateHorizontalEncoder();
                }
                robot.setHorizontalSlidesPower(-0.8);
                horizontalPositionUpdated = true;
            }
            else {
//                robot.setHorizontalSlidesPower(0);
                if(horizontalPositionUpdated) {
                    robot.brakeHorizontalSlides(0.1); // TODO: CHECK THIS
                    horizontalPositionUpdated = false;
                }
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
            if(deliverRaiseTask != null) {
                deliverRaiseTask.cancel();
                deliverRaiseTask = null;
            } else if(deliverTask != null){
                deliverTask.cancel();
                deliverTask = null;
            }
            robot.setVerticalSlidePowerWithoutEncoder(gp2.dpadDown()? -0.9 : 1.0);
        } else if(deliverRaiseTask == null && deliverTask == null) {
            robot.setVerticalSlidePowerWithEncoder(robot.getVerticalSlidesHeight() < 0.5? 0:VERTICAL_STAND_POWER);
        }

        // Bucket
        if(gp1.onceX()) {
            robot.setBucketServoPosition(BucketTask.Position.FLAT);
        } else if(!gp1.back() && gp1.onceA()) {
            robot.setBucketServoPosition(BucketTask.Position.DELIVER);
            if(robot.getVerticalSlidesHeight() >= 12) {
                cycleSum += cycleTimer.seconds(); cycleCnt++; cycleAvg = cycleSum/cycleCnt;
                cycleTimer.reset();
            }
        } else if(gp1.onceY()) {
            robot.setBucketServoPosition(BucketTask.Position.TRANSFER);
        }

        // Claw
        if(gp1.onceB()){
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

        if(-gp2.rightStickY()>0.5) {
            robot.setIntakeArmPos(IntakeArmTask.Position.TRANSFER);
        } else if(gp2.onceRightStickButton()) {
            robot.setIntakeArmPos(IntakeArmTask.Position.CLEAR);
        } else if(-gp2.rightStickY()<-0.5) {
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
