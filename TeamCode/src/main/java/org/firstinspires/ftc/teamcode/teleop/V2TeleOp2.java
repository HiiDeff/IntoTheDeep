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
import org.firstinspires.ftc.teamcode.task.ClawTask;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.V2DiffyTask;
import org.firstinspires.ftc.teamcode.task.VerticalSlidesTask;
import org.firstinspires.ftc.teamcode.util.GamePad;

//@Config
@TeleOp(name = "V2 Neel Requested TeleOp")
@Config

public class V2TeleOp2 extends LinearOpMode {

    public static double VERTICAL_STAND_POWER = 0.001; //chng to private
    public static double VERTICAL_SLIDE_MAX_POWER = 0.5;
    public static double MIN_DRIVE_POWER = 0.01;
    public static double HORIZONTALSLIDE_DELTA = 2;

    private V2RobotBase robot;
    private GamePad gp1, gp2;
    private ElapsedTime intakeTimer;
    private boolean horizontalSlideExtended, clawOpen;

    private Task diffyTask;
    private Task horizontalSlidesTask;
    private Task verticalSlidesTask;
    private Task resetTask;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = (V2RobotBase) RobotFactory.createRobot(hardwareMap);
        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);

        robot.teleOpInit();
        waitForStart();
        ElapsedTime cycleTimer = new ElapsedTime();
        double lastTime = cycleTimer.milliseconds();
        while (opModeIsActive()) {
            driveRobot();
            runIntake();
            runDelivery();
            runHorizontalSlides();

            if(diffyTask==null) runDiffy();
            if(resetTask==null) runReset();

            update();

            telemetry.addData("colors", robot.getIntakeColorToString());

            telemetry.addData("cycle time", cycleTimer.milliseconds() - lastTime);
            telemetry.addData("pose (may be slightlyyyy inaccurate)", robot.pose);
            lastTime = cycleTimer.milliseconds();
        }
    }
    private void update() {
        if (horizontalSlidesTask != null && horizontalSlidesTask.perform()) {
            horizontalSlidesTask = null;
        }
        if (verticalSlidesTask != null && verticalSlidesTask.perform()) {
            verticalSlidesTask = null;
        }
        if(diffyTask != null && diffyTask.perform()) {
            diffyTask = null;
        }
        if(resetTask != null && resetTask.perform()) {
            resetTask = null;
        }
        gp1.update();
        gp2.update();
        telemetry.update();
        robot.updateEverything();
    }

    private void runReset() {
        if(gp1.onceRightBumper() || gp2.onceRightBumper()) {
            resetTask = new SeriesTask(
                    new ParallelTask(
                            new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.LEFT, V2DiffyTask.DiffyPosition.TRANSFER),
                            new V2DiffyTask(robot, V2DiffyTask.WhichDiffy.RIGHT, V2DiffyTask.DiffyPosition.TRANSFER)
                    ),
                    new ParallelTask(
                            new VerticalSlidesTask(robot, 0, 0.9, 3000),
                            new HorizontalSlidesTask(robot, V2RobotBase.HORIZONTAL_SLIDE_TRANSFER_POS, 0.9,500),
                            new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER)
                    ),
                    new SleepTask(1000)
            );
        }
    }

    private void runDiffy() {
        if(gp1.onceX()) { //delivery slides up and diffy carry
            //     robot.setDefaultVerticalSlidesHeight(VERTICAL_SLIDE_MAX_POWER);
            robot.setDiffyPosition(V2DiffyTask.DiffyPosition.FLAT); //ALSO BAD CONVENTION TODO: FIX
        }
        else if(gp1.onceA()) { //diffy transfer
            robot.setDiffyPosition(V2DiffyTask.DiffyPosition.TRANSFER);
        }
        else if(gp1.onceY()) { //diffy deliver
            robot.setDiffyPosition(V2DiffyTask.DiffyPosition.DELIVER);
        }
        if(gp2.onceB()){
            if(clawOpen) {
                robot.setClawPosition(robot.getClawPosition(ClawTask.ClawPosition.CLOSE));
            }else{
                robot.setClawPosition(robot.getClawPosition(ClawTask.ClawPosition.OPEN));
            }
            clawOpen = !clawOpen;
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
//            if (horizontalSlidesTask != null) {
//                horizontalSlidesTask.cancel();
//                horizontalSlidesTask = null;
//            } else {
//                if (!horizontalSlideExtended) {
//                    horizontalSlidesTask = new HorizontalSlidesTask(robot, 18.0, 0.8, 2000);
//                } else {
//                    horizontalSlidesTask = new HorizontalSlidesTask(robot, 147 / 32.91, 0.8, 2000);
//                }
//                horizontalSlideExtended = !horizontalSlideExtended;
//            }
//        }
//        else if(gp2.dpadUp()) {
//            if (horizontalSlidesTask != null)
//            {
//                horizontalSlidesTask.cancel();
//            }
//            double currPos = robot.getHorizontalSlideDist();
//            horizontalSlidesTask = new HorizontalSlidesTask(robot, currPos + HORIZONTALSLIDE_DELTA, 0.6, 2000);
//
//        }
//        else if(gp2.dpadDown()) {
//            if (horizontalSlidesTask != null)
//            {
//                horizontalSlidesTask.cancel();
//            }
//            double currPos = robot.getHorizontalSlideDist();
//            horizontalSlidesTask = new HorizontalSlidesTask(robot, currPos - HORIZONTALSLIDE_DELTA, 0.6, 2000);
//        }
    }

    private void driveRobot() {
        double x = 0, y = 0, a = 0;
        if (gp1.dpadLeft() || gp1.dpadRight()) {
            a = 0.9 * (gp1.dpadLeft() ? 1 : -1);
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

        if(gp2.dpadUp()){
            robot.setVerticalSlidePower(1.0);
        } else if(gp2.dpadDown()) {
            robot.setVerticalSlidePower(-1.0);
        }
        else if(resetTask == null){
            robot.setVerticalSlidePower(VERTICAL_STAND_POWER);
        }
    }

    private void runIntake() {

        if(gp2.onceY()) {
            robot.setIntakeArmUp();
        }
        else if(gp2.onceX()) {
            robot.setIntakeArmClear();
        }
        else if(gp2.onceA()) {
            robot.setIntakeArmDown();
        }

//        if(gp2.onceLeftBumper()){
//            robot.toggleHorizontalSlides();
//        }
//        if(gp2.onceB()) {
//            robot.setIntakeArmPos(robot.getIntakeArmPos(IntakeArmTask.Position.CLEAR));
//        }
//        if(gp2.onceRightBumper()){
//            robot.toggleIntakeArm();
//        }

        if(gp2.rightTrigger()>0.1){
            robot.runIntakeIn();
        } else if(gp2.leftTrigger()>0.1) {
            robot.runIntakeOut();
        } else{
            robot.stopIntake();
        }
    }

}
