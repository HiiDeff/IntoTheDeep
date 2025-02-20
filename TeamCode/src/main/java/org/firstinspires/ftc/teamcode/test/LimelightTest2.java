package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.V3Robot.V3Robot;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.util.limelight.Coordinates;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightConfig;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightProcessor;

@TeleOp(name = "Limelight Tester 2", group = "Test")
@Config
public class LimelightTest2 extends LinearOpMode {

    public static LimelightConfig LLConfig = new LimelightConfig(320, 240,
            -30, 54,41,
            7.5,-3.5,12.75);
    public static double objectCenterHeightInch = 0.0;
    private LimelightProcessor limelightProcessor;

    public V3Robot robot;
    public static boolean updateHorizontalSlidesPosition = false;
    public static double prevY;
    public static int sleepTime = 300;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = (V3Robot) RobotFactory.createRobot(hardwareMap);
        limelightProcessor = new LimelightProcessor(robot.limelight, LLConfig);

        robot.limelight.pipelineSwitch(1);
        robot.limelight.start();
        robot.setIntakeArmPos(IntakeArmTask.Position.TRANSFER);

        waitForStart();

        while (opModeIsActive()) {

            LLStatus status = robot.limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());


            Coordinates objectPosition = limelightProcessor.getObjectPosition(objectCenterHeightInch);
            telemetry.addData("objectXPos",objectPosition.x);
            telemetry.addData("objectYPos",objectPosition.y);
            telemetry.update();



            if(updateHorizontalSlidesPosition) {
                robot.setHorizontalSlideDist(objectPosition.y, 1.0);
                updateHorizontalSlidesPosition = false;
            }else {
                prevY = objectPosition.y;
            }

        }
        robot.limelight.stop();
    }

}
