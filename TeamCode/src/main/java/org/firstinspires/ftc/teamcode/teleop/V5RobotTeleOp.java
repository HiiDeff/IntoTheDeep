package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.V5Robot.V5Robot;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.v5_tasks.V5DeliveryClawTask;
import org.firstinspires.ftc.teamcode.task.v5_tasks.V5DiffyTask;
import org.firstinspires.ftc.teamcode.task.v5_tasks.V5HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.v5_tasks.V5IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.v5_tasks.V5IntakeClawTask;
import org.firstinspires.ftc.teamcode.task.v5_tasks.V5IntakePivotTask;
import org.firstinspires.ftc.teamcode.task.v5_tasks.V5IntakeWristTask;
import org.firstinspires.ftc.teamcode.task.v5_tasks.V5TurretTask;
import org.firstinspires.ftc.teamcode.util.GamePad;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightConfig;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightProcessorBase;

@TeleOp(name = "V5 Robot TeleOp")
public class V5RobotTeleOp extends LinearOpMode {

    public static int pipelineNo = 6;
    private Task task = null;
    private IntakeState intakeState = IntakeState.NEUTRAL;


    private V5Robot robot;
    private GamePad gp1, gp2;

    private boolean clawOpen, intakeClawVertical = true;

    public static double MAX_SLIDE_EXTENSION_INCH = 21.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = (V5Robot) RobotFactory.createRobot(hardwareMap);
        robot.limelight.pipelineSwitch(pipelineNo);
        robot.teleOpInit();
        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);
        waitForStart();
        robot.limelight.start();

        while (opModeIsActive()) {

            driveRobot();
            runIntake();
            runHorizontalSlides();
            checkPreset();
            update();
        }
        robot.limelight.stop();
    }
    private void checkPreset() {

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
        if(gp1.onceRightBumper()) {
            if(task!=null) {
                task.cancel();
            }
            task = new V5HorizontalSlidesTask(robot, 20, 1.0, 1500);
        } else if(gp1.onceLeftBumper()) {
            if(task!=null) {
                task.cancel();
            }
            task = new V5HorizontalSlidesTask(robot, 10, 1.0, 1500);
        }
//        if(task!=null) return;
//        if ((robot.getHorizontalSlideDistInch() < MAX_SLIDE_EXTENSION_INCH && -gp2.leftStickY() > 0.3) || (-gp2.leftStickY() < -0.3)) {
//            robot.setHorizontalSlidesPowerWithoutEncoder(-gp2.leftStickY() > 0.3 ? 1.0 : -1.0);
//            if (gp2.rightBumper()) {
//                robot.resetHorizontalAndVerticalSlidesEncoderPositions();
//            } else {
//                robot.updateLastHorizontalEncoder();
//                horizontalPositionUpdated = true;
//            }
//        }
//        else{
//            if (horizontalPositionUpdated) {
//                robot.brakeHorizontalSlides(0.01); // TODO: CHECK THIS
//                horizontalPositionUpdated = false;
//            }
//        }
    }

    private void runIntake(){

        if(gp2.onceRightTrigger()) {
            if(task != null) task.cancel();
            if(intakeState == IntakeState.NEUTRAL) {
                //go to hover
                task = new SeriesTask(
                        new V5DeliveryClawTask(robot, V5DeliveryClawTask.Position.OPEN),
                        new ParallelTask(
                            new V5IntakeArmTask(robot, V5IntakeArmTask.Position.HOVER),
                            new V5IntakeClawTask(robot, V5IntakeClawTask.Position.OPEN),
                            new V5IntakePivotTask(robot, V5IntakePivotTask.Position.VERTICAL),
                            new V5IntakeWristTask(robot, V5IntakeWristTask.Position.HOVER),
                            new V5TurretTask(robot, V5TurretTask.Position.STRAIGHT),
                            new V5DiffyTask(robot, V5DiffyTask.WhichDiffy.LEFT, V5DiffyTask.Position.TRANSFER),
                            new V5DiffyTask(robot, V5DiffyTask.WhichDiffy.RIGHT, V5DiffyTask.Position.TRANSFER)
                        )
                );
                intakeClawVertical = true;
                intakeState = IntakeState.HOVER;
            } else if(intakeState == IntakeState.TRANSFER) {
                //go to neutral
                task = new ParallelTask(
                        new V5TurretTask(robot, V5TurretTask.Position.STRAIGHT),
                        new V5IntakeArmTask(robot, V5IntakeArmTask.Position.BACK),
                        new V5IntakeWristTask(robot, V5IntakeWristTask.Position.TRANSFER),
                        new V5IntakePivotTask(robot, V5IntakePivotTask.Position.VERTICAL)
                );
                intakeClawVertical = true;
                intakeState = IntakeState.NEUTRAL;
            } else if(intakeState == IntakeState.HOVER) {
                //try to intake
                task = new SeriesTask(
                        new ParallelTask(
                                new V5IntakeArmTask(robot, V5IntakeArmTask.Position.DOWN),
                                new V5IntakeWristTask(robot, V5IntakeWristTask.Position.DOWN)
                        ),
                        new ParallelTask(
                                new V5IntakeClawTask(robot, V5IntakeClawTask.Position.CLOSE)
                        ),
                        new SleepTask(400),
                        new ParallelTask(
                                new V5IntakeArmTask(robot, V5IntakeArmTask.Position.HOVER),
                                new V5IntakeWristTask(robot, V5IntakeWristTask.Position.HOVER)
                        )
                );
                intakeState = IntakeState.HOVER;
            }
        } else if(gp2.onceLeftTrigger()) {
            if(intakeState == IntakeState.HOVER) {
                if(task != null) task.cancel();
                //go to transfer
                task = new SeriesTask(
                        new ParallelTask(
                                new V5DeliveryClawTask(robot, V5DeliveryClawTask.Position.OPEN),
                                new V5DiffyTask(robot, V5DiffyTask.WhichDiffy.LEFT, V5DiffyTask.Position.TRANSFER),
                                new V5DiffyTask(robot, V5DiffyTask.WhichDiffy.RIGHT, V5DiffyTask.Position.TRANSFER)
                        ),
                        new ParallelTask(
                                new V5HorizontalSlidesTask(robot, 0, 1.0, 1500),
                                new V5IntakeArmTask(robot, V5IntakeArmTask.Position.TRANSFER),
                                new V5IntakeClawTask(robot, V5IntakeClawTask.Position.CLOSE),
                                new V5IntakePivotTask(robot, V5IntakePivotTask.Position.VERTICAL),
                                new V5IntakeWristTask(robot, V5IntakeWristTask.Position.TRANSFER),
                                new V5TurretTask(robot, V5TurretTask.Position.STRAIGHT)
                        ),
                        new V5DeliveryClawTask(robot, V5DeliveryClawTask.Position.CLOSE),
                        new V5IntakeClawTask(robot, V5IntakeClawTask.Position.OPEN),
                        new ParallelTask(
                                new V5DiffyTask(robot, V5DiffyTask.WhichDiffy.LEFT, V5DiffyTask.Position.FLAT),
                                new V5DiffyTask(robot, V5DiffyTask.WhichDiffy.RIGHT, V5DiffyTask.Position.FLAT)
                        )
                );
                intakeClawVertical = true;
                intakeState = IntakeState.TRANSFER;
            }
        }

        if (gp2.onceA()) {
            robot.setDeliveryClawPos(clawOpen ? V5DeliveryClawTask.Position.CLOSE : V5DeliveryClawTask.Position.OPEN);
            clawOpen = !clawOpen;
        }

        if (gp2.onceRightTrigger()) {
            robot.setIntakePivotPos(intakeClawVertical? V5IntakePivotTask.Position.HORIZONTAL : V5IntakePivotTask.Position.VERTICAL);
            intakeClawVertical = ! intakeClawVertical;
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

    private enum IntakeState {
        NEUTRAL,
        TRANSFER,
        HOVER
    }

}
