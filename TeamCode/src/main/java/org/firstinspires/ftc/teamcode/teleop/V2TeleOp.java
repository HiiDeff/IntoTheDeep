package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.V2RobotBase;
import org.firstinspires.ftc.teamcode.task.ClawTask;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.V2DiffyTask;
import org.firstinspires.ftc.teamcode.util.GamePad;

//@Config
@TeleOp(name = "V2 TeleOp")
@Config

public class V2TeleOp extends LinearOpMode {

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

            if(diffyTask==null) runDiffy();
            if(horizontalSlidesTask==null) runHorizontalSlides();

            update();

            telemetry.addData("cycle time", cycleTimer.milliseconds() - lastTime);
            telemetry.addData("pose (may be slightlyyyy inaccurate)", robot.pose);
            lastTime = cycleTimer.milliseconds();
        }
    }
    private void update() {
        if (horizontalSlidesTask != null && horizontalSlidesTask.perform()) {
            horizontalSlidesTask = null;
        }
        if(diffyTask != null && diffyTask.perform()) {
            diffyTask = null;
        }
        gp1.update();
        gp2.update();
        telemetry.update();
        robot.updateEverything();
    }

    private void runDiffy() {
        if(gp2.onceX()) { //delivery slides up and diffy carry
            robot.setDefaultVerticalSlidesHeight(VERTICAL_SLIDE_MAX_POWER);
            robot.setDiffyPosition(V2DiffyTask.DiffyPosition.CARRY); //ALSO BAD CONVENTION TODO: FIX
        }
        else if(gp2.onceY()) { //diffy transfer
            robot.setDiffyPosition(V2DiffyTask.DiffyPosition.TRANSFER);
        }
        else if(gp2.onceA()) { //diffy deliver
            robot.setDiffyPosition(V2DiffyTask.DiffyPosition.DELIVER);
        }
        if(gp1.onceX()){
            if(clawOpen) {
                robot.setClawPosition(robot.getClawPosition(ClawTask.ClawPosition.CLOSE));
            }else{
                robot.setClawPosition(robot.getClawPosition(ClawTask.ClawPosition.OPEN));
            }
            clawOpen = !clawOpen;
        }
    }
    private void runHorizontalSlides() {
        if (gp2.onceLeftBumper()) {
            if (horizontalSlidesTask != null) {
                horizontalSlidesTask.cancel();
                horizontalSlidesTask = null;
            } else {
                if (!horizontalSlideExtended) {
                    horizontalSlidesTask = new HorizontalSlidesTask(robot, 18.0, 0.8, 2000);
                } else {
                    horizontalSlidesTask = new HorizontalSlidesTask(robot, 147 / 32.91, 0.8, 2000);
                }
                horizontalSlideExtended = !horizontalSlideExtended;
            }
        }
        else if(gp2.dpadUp()) {
            if (horizontalSlidesTask != null)
            {
                horizontalSlidesTask.cancel();
            }
            double currPos = robot.getHorizontalSlideDist();
            horizontalSlidesTask = new HorizontalSlidesTask(robot, currPos + HORIZONTALSLIDE_DELTA, 0.6, 2000);

        }
        else if(gp2.dpadDown()) {
            if (horizontalSlidesTask != null)
            {
                horizontalSlidesTask.cancel();
            }
            double currPos = robot.getHorizontalSlideDist();
            horizontalSlidesTask = new HorizontalSlidesTask(robot, currPos - HORIZONTALSLIDE_DELTA, 0.6, 2000);
        }
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

        if(Math.abs(gp2.leftStickY()) > MIN_DRIVE_POWER){
            robot.setVerticalSlidePower(-gp2.leftStickY()*VERTICAL_SLIDE_MAX_POWER, VERTICAL_STAND_POWER);
        }
        else{
            robot.setVerticalSlidePower(VERTICAL_STAND_POWER);
        }
    }

    private void runIntake() {
//        if(gp2.onceLeftBumper()){
//            robot.toggleHorizontalSlides();
//        }
        if(gp2.onceB()) {
            robot.setIntakeArmPos(robot.getIntakeArmPos(IntakeArmTask.Position.CLEAR));
        }
        if(gp2.onceRightBumper()){
            robot.toggleIntakeArm();
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
