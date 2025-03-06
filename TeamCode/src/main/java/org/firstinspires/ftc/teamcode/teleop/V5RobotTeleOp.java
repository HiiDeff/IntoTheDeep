package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.V5Robot.V5Robot;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.V5HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.V5IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.V5IntakeClawTask;
import org.firstinspires.ftc.teamcode.task.V5IntakeWristTask;
import org.firstinspires.ftc.teamcode.util.GamePad;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightConfig;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightProcessorBase;

@TeleOp(name = "V5 Robot TeleOp")
public class V5RobotTeleOp extends LinearOpMode {

    public static LimelightConfig LLConfig = new LimelightConfig(320, 240,
            -30, 54,41,
            7.7,4.0,13.25); //ROBOT 4 MEASUREMENTS
    public static double objectCenterHeightInch = 0.0;
    private LimelightProcessorBase limelightProcessorBase;
    public static int pipelineNo = 1;
    public static double distBeforeSampleInch = 13.0;
    private Task task = null;

    public ElapsedTime elapsedTime = new ElapsedTime();

    private V5Robot robot;
    private GamePad gp1, gp2;
    private ElapsedTime cycleTimer;
    private double cycleSum, cycleCnt, cycleAvg;

    private boolean clawOpen, horizontalPositionUpdated, hangUp, swipeStickOut;

    public static double MAX_SLIDE_EXTENSION_INCH = 21.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = (V5Robot) RobotFactory.createRobot(hardwareMap);
        robot.teleOpInit();
        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);
        waitForStart();
        cycleTimer = new ElapsedTime();
//        limelightProcessor = new LimelightProcessor(robot.limelight, LLConfig);

//
//        robot.limelight.pipelineSwitch(pipelineNo);
//        robot.limelight.start();
        robot.setIntakeArmPos(V5IntakeArmTask.Position.DOWN);

        while (opModeIsActive()) {

            driveRobot();
            runIntake();
            runHorizontalSlides();
            checkPreset();
            update();
            elapsedTime.reset();
        }
//        robot.limelight.stop();
    }
    private void checkPreset() {
        if(gp1.onceA()) {
            if(task != null) {
                task.cancel();
                task = null;
            } else{
//                task = new SeriesTask(
//                        new V5IntakeArmTask(robot, V5IntakeArmTask.Position.TRANSFER),
//                        new ParallelTask(
//                                new RuntimeDrivingTask(robot, builder -> {
//                                    return builder
//                                            .turnTo(currentRobotPosition.getHeading(AngleUnit.RADIANS)-objectPosition.angle)
//                                            .build();
//                                }
//                                ),
//                                new HorizontalSlidesTask(robot, absoluteDist - distBeforeSampleInch,1.0,1500)
//                        ),
//                        new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
//                        new ParallelTask(
//                                new HorizontalSlidesTask(robot, absoluteDist-distBeforeSampleInch+forwardDist, 1.0, 1500),
//                                new UnboundedIntakeTask(robot, 1.0)
//                        ),
//                        new ParallelTask(
//                                new HorizontalSlidesTask(robot, 0, 1.0, 1500),
//                                new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER),
//                                new UnboundedIntakeTask(robot, 0.5),
//                                new RuntimeDrivingTask(robot, builder ->{
//                                    return builder
//                                            .turnTo(currentRobotPosition.getHeading(AngleUnit.RADIANS))
//                                            .build();
//                                }
//                                )
//                        ),
//                        new IntakeArmTask(robot, IntakeArmTask.Position.DOWN)
//                );
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

        robot.setDrivePowers(new PoseVelocity2d(new Vector2d(x, y), a));
    }

    private void runHorizontalSlides() { //TODO: check this logic
        if(task!=null) return;
        if ((robot.getHorizontalSlideDistInch() < MAX_SLIDE_EXTENSION_INCH && -gp2.leftStickY() > 0.3) || (-gp2.leftStickY() < -0.3)) {
            robot.setHorizontalSlidesPowerWithoutEncoder(-gp2.leftStickY() > 0.3 ? 1.0 : -1.0);
            if (gp2.rightBumper()) {
                robot.resetHorizontalAndVerticalSlidesEncoderPositions();
            } else {
                robot.updateLastHorizontalEncoder();
                horizontalPositionUpdated = true;
            }
        }
        else{

            if (horizontalPositionUpdated) {
                robot.brakeHorizontalSlides(0.01); // TODO: CHECK THIS
                horizontalPositionUpdated = false;
            }
        }
    }

    private void runIntake(){
        if (-gp2.rightStickY()>0.3) {
            robot.setIntakeArmPos(V5IntakeArmTask.Position.TRANSFER);
        } else if (-gp2.rightStickY()<-0.3) {
            robot.setIntakeArmPos(V5IntakeArmTask.Position.DOWN);
        }

        if (gp2.onceRightTrigger()) {
            robot.setClawPosition(clawOpen ? V5IntakeClawTask.ClawPosition.CLOSE : V5IntakeClawTask.ClawPosition.OPEN);
            clawOpen = !clawOpen;
        }
        if(gp2.onceY()) {
            if(robot.getHorizontalSlideDistInch()>1.0){
                task = new V5HorizontalSlidesTask(robot, 0.0, 1.0, 1500);
            } else {
                task = new V5HorizontalSlidesTask(robot, 20.0, 1.0, 1500);
            }
        }

        if(gp2.leftBumper()){
            robot.setWristPos(V5IntakeWristTask.WristPosition.HORIZONTAL);
        }
        else if(gp2.rightBumper()){
            robot.setWristPos(V5IntakeWristTask.WristPosition.VERTICAL);
        }
    }

    private void update() {

//        LLStatus status = robot.limelight.getStatus();
//        telemetry.addData("Name", "%s",
//                status.getName());
//        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                status.getTemp(), status.getCpu(),(int)status.getFps());
//        telemetry.addData("Pipeline", "Index: %d, Type: %s",
//                status.getPipelineIndex(), status.getPipelineType());
//
//        Coordinates objectPosition = limelightProcessor.getObjectPosition(objectCenterHeightInch);
//        telemetry.addData("objectXPos",objectPosition.x);
//        telemetry.addData("objectYPos",objectPosition.y);
//        telemetry.addData("objectAngleRadians",objectPosition.angle);

        if(task != null && task.perform())  task = null;
        gp1.update();
        gp2.update();
        robot.updateEverything();
    }

}
