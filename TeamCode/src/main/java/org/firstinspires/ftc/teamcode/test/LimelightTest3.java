package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.UnboundedIntakeTask;
import org.firstinspires.ftc.teamcode.util.GamePad;
import org.firstinspires.ftc.teamcode.util.limelight.Coordinates;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightConfig;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightProcessorBase;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightSampleDetector;

import java.util.List;

@TeleOp(name = "Limelight Tester 3", group = "Test")
@Config
public class LimelightTest3 extends LinearOpMode {

    public static LimelightConfig LLConfig = new LimelightConfig(320, 240,
            -30, 54,41,
            7.7,4.0,13.75); //ROBOT 4 MEASUREMENTS
    private LimelightSampleDetector limelightProcessorBase;

    public RobotBase robot;
    private GamePad gamepad;
    public static boolean updateHorizontalSlidesPosition = false;
    public static double prevY;
    public static int pipelineNo = 1, sleepTime = 300;
    public static double distBeforeSampleInch = 13.0, forwardDist = 5.0;
    private Task task = null;
    public MultipleTelemetry multipleTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepad = new GamePad(gamepad1);
        robot = (RobotBase) RobotFactory.createRobot(hardwareMap);
        limelightProcessorBase = new LimelightSampleDetector(robot.limelight, LLConfig);

        robot.limelight.pipelineSwitch(pipelineNo);
        robot.limelight.start();
        robot.setIntakeArmPos(IntakeArmTask.Position.DOWN);

        waitForStart();

        while (opModeIsActive()) {

            LLStatus status = robot.limelight.getStatus();
            multipleTelemetry.addData("Name", "%s",
                    status.getName());
            multipleTelemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            multipleTelemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            Coordinates objectPosition = limelightProcessorBase.getObjectPosition(0.5);
            multipleTelemetry.addData("objectXPos",objectPosition.x);
            multipleTelemetry.addData("objectYPos",objectPosition.y);
            multipleTelemetry.addData("objectAngleRadians",objectPosition.angle);

            LLResult result = robot.limelight.getLatestResult();
            if(result!=null && result.isValid()){
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                LLResultTypes.ColorResult firstResult = colorResults.get(0);
                List<List<Double>> temp = firstResult.getTargetCorners();
                for(List<Double> i: temp) {
                    multipleTelemetry.addData("corner", i);
                }
                multipleTelemetry.addData("size", temp.size());
            }
            if(gamepad.onceX()) {
                Pose2D currentRobotPosition = robot.pinpoint.getPosition();
                double absoluteDist = Math.sqrt(objectPosition.x*objectPosition.x + objectPosition.y*objectPosition.y);

                task = new SeriesTask(
                        new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER),
                        new ParallelTask(
                                new RuntimeDrivingTask(robot, builder -> {
                                    return builder
                                            .turnTo(currentRobotPosition.getHeading(AngleUnit.RADIANS)-objectPosition.angle)
                                            .build();
                                }
                                ),
                                new HorizontalSlidesTask(robot, absoluteDist - distBeforeSampleInch,1.0,1500)
                        ),
                        new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                        new ParallelTask(
                                new HorizontalSlidesTask(robot, absoluteDist-distBeforeSampleInch+forwardDist, 1.0, 1500),
                                new UnboundedIntakeTask(robot, 1.0)
                        ),
                        new ParallelTask(
                                new HorizontalSlidesTask(robot, 0, 1.0, 1500),
                                new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER),
                                new UnboundedIntakeTask(robot, 0.5),
                                new RuntimeDrivingTask(robot, builder ->{
                                    return builder
                                            .turnTo(currentRobotPosition.getHeading(AngleUnit.RADIANS))
                                            .build();
                                }
                                )
                        ),
                        new IntakeArmTask(robot, IntakeArmTask.Position.DOWN)
                );
            }


            if(task != null && task.perform())  task = null;
            gamepad.update();
            robot.updateEverything();
            multipleTelemetry.update();
        }
        robot.limelight.stop();
    }

}
