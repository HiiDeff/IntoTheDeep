package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.V3Robot;
import org.firstinspires.ftc.teamcode.task.BucketTask;
import org.firstinspires.ftc.teamcode.task.ClawTask;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.IntakeTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.TimedHorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.UnboundedIntakeTask;
import org.firstinspires.ftc.teamcode.task.VerticalSlidesTask;
import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "V3 TeleOp")
@Config

public class V3RobotTeleOp extends LinearOpMode {

    public MultipleTelemetry multipleTelemetry;

    public static double VERTICAL_STAND_POWER = 0.2;
    public static double VERTICAL_SLIDE_MAX_POWER = 1.0;
    public static double MIN_DRIVE_POWER = 0.01;
    public static double DELIVERY_CHAMBER_HEIGHT = 20.5;

    private V3Robot robot;
    private GamePad gp1, gp2;
    private ElapsedTime cycleTimer;
    private double cycleSum, cycleCnt, cycleAvg;
    private boolean clawOpen, horizontalPositionUpdated;

    private Task submersibleTask,
            transferTask,
            deliverChamberTask,
            deliverTask;
    ;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = (V3Robot) RobotFactory.createRobot(hardwareMap);
        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);

        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
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
        }if (deliverChamberTask != null && deliverChamberTask.perform()) {
            deliverChamberTask = null;
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
        telemetry.addData("vertical slide position", robot.getVerticalSlidesHeightInch());
        telemetry.addData("horizontal slide position", robot.getHorizontalSlideDist());

        multipleTelemetry.addData("current bucket servo position", robot.bucketProfiler.getCurPos());
        multipleTelemetry.addData("target bucket servo position", robot.bucketProfiler.getTargetPos());
        telemetry.update();
        multipleTelemetry.update();
    }

    private void checkPreloads() {

        if(submersibleTask!=null || transferTask != null || deliverChamberTask != null) {
            return;
        }

        // Submersible Intake Preset
        if(gp2.onceY()){
            submersibleTask = new SeriesTask(
                    new ParallelTask(
                            new IntakeArmTask(robot, IntakeArmTask.Position.CLEAR),
                            new HorizontalSlidesTask(robot, 9.0, 1.0, 500)
                    ),
                    new IntakeArmTask(robot, IntakeArmTask.Position.DOWN)
            );
        }

        // Transfer Preset
        else if(gp2.onceX()) {
            transferTask = new SeriesTask(
                    new ParallelTask(
                            new BucketTask(robot, BucketTask.Position.TRANSFER),
                            new ClawTask(robot, ClawTask.ClawPosition.OPEN)
                    ),
                    new ParallelTask(
                            new VerticalSlidesTask(robot, 0, 1.0, 2000),
                            new SeriesTask(
                                    new ParallelTask(
                                            new HorizontalSlidesTask(robot, robot.getHorizontalTransferPos(), 1.0,1000),
                                            new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER)
                                    ),
                                    new ParallelTask(
                                            new TimedHorizontalSlidesTask(robot, robot.getHorizontalTransferPos(), 1.0, 2000),
                                            new IntakeTask(robot, 2000, -1.0)
                                    )
                            )
                    )
            ); clawOpen = true;
        }

        // Deliver Raise Preset
        else if(!gp2.start() && gp2.onceB()) {
            deliverChamberTask = new SeriesTask(
                    new ClawTask(robot, ClawTask.ClawPosition.CLOSE),
                    new ParallelTask(
                            new VerticalSlidesTask(robot, DELIVERY_CHAMBER_HEIGHT, 1.0, 700),
                            new BucketTask(robot, BucketTask.Position.CHAMBER_DELIVER)
                    )
            );
            clawOpen = false;
        }

//        // Deliver Preset
//        else if(deliverTask == null && (gp2.onceB())) {
//            deliverTask = new VerticalSlidesTask(robot, 0, 1.0, 1500);
//
////                    new SeriesTask(
////                    new BucketTask(robot, BucketTask.Position.DELIVER),
////                    new SleepTask(1000),
////                    new ParallelTask(
////                            new BucketTask(robot, BucketTask.Position.TRANSFER),
////                            new SeriesTask(
////                                    new SleepTask(300),
////                                    new VerticalSlidesTask(robot, 0, 1.0, 1500)
////                            )
////                    )
////            );
//            if(robot.getVerticalSlidesHeight() >= 12) {
//                cycleSum += cycleTimer.seconds(); cycleCnt++; cycleAvg = cycleSum/cycleCnt;
//                cycleTimer.reset();
//            }
//        }
    }

    private void runHorizontalSlides() {

        if(-gp2.leftStickY()>0.3 || -gp2.leftStickY()<-0.3) {
            if(submersibleTask != null){
                submersibleTask.cancel();
                submersibleTask = null;
            }
            if(transferTask != null) {
                transferTask.cancel();
                transferTask = null;
            }

            robot.setHorizontalSlidesPowerWithoutEncoder(-gp2.leftStickY()>0.3 ? 0.8 : -0.8);

            if(gp2.rightBumper()){
                robot.horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.horizontalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else{
                robot.updateHorizontalEncoder();
                horizontalPositionUpdated = true;
            }
        }
        else if(submersibleTask == null && transferTask == null){
            if(horizontalPositionUpdated) {
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
            a = -gp1.rightStickX() * 1.0;
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
        if(transferTask == null && (gp2.dpadUp() || gp2.dpadDown())){
            if(deliverChamberTask != null) {
                deliverChamberTask.cancel();
                deliverChamberTask = null;
            } else if(deliverTask != null){
                deliverTask.cancel();
                deliverTask = null;
            }
            if(gp2.dpadUp()) {
                robot.setVerticalSlidePowerWithoutEncoder(1.0);
            } else{
                robot.setVerticalSlidePowerWithoutEncoder(-0.3);
            }
        } else if(deliverChamberTask == null && deliverTask == null && transferTask == null) {
            robot.setVerticalSlidePowerWithoutEncoder(robot.getVerticalSlidesHeightInch() < 1.0? 0:VERTICAL_STAND_POWER);
        }

        // Bucket
        if(gp1.onceB()) {
            robot.setBucketServoPosition(BucketTask.Position.FLAT);
            robot.setClawPosition(ClawTask.ClawPosition.OPEN); clawOpen = true;
        } else if(gp1.onceX()){
            robot.setBucketServoPosition(BucketTask.Position.CARRY);
            robot.setClawPosition(ClawTask.ClawPosition.OPEN); clawOpen = true;
        }else if(!gp1.back() && gp1.onceY()) {
            robot.setBucketServoPosition(BucketTask.Position.DELIVER);
            robot.setClawPosition(ClawTask.ClawPosition.OPEN); clawOpen = true;
            if(robot.getVerticalSlidesHeightInch() >= 12) {
                cycleSum += cycleTimer.seconds(); cycleCnt++; cycleAvg = cycleSum/cycleCnt;
                cycleTimer.reset();
            }
        } else if(!gp1.back() && !gp1.start() && gp1.onceA()) {
            robot.setBucketServoPosition(BucketTask.Position.TRANSFER);
            robot.setClawPosition(ClawTask.ClawPosition.OPEN); clawOpen = true;
        }

        // Claw
        if(gp2.onceA()){
            if(clawOpen) {
                robot.setClawPosition(ClawTask.ClawPosition.CLOSE);
            } else {
                robot.setClawPosition(ClawTask.ClawPosition.OPEN);
            }
            clawOpen = !clawOpen;
        }

        // Hang
        if(gp1.back() && gp1.onceY()) {
            robot.releaseHang();
        } else if(gp1.back() && gp1.onceA()) {
            robot.lowerHang();
            robot.setBucketServoPosition(BucketTask.Position.FLAT);
            robot.setClawPosition(ClawTask.ClawPosition.OPEN);
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
