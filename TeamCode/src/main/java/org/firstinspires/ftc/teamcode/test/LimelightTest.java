package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.limelight.Coordinates;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightConfig;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightProcessor;

@TeleOp(name = "Limelight Tester", group = "Test")
@Config
public class LimelightTest extends LinearOpMode {

    public static LimelightConfig LLConfig = new LimelightConfig(320, 240,
            -30, 54,41,
            7.5,-3.5,12.75); // ROBOT 3
    public static double objectCenterHeightInch = 0.0;
    private LimelightProcessor limelightProcessor;
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelightProcessor = new LimelightProcessor(limelight, LLConfig);

        limelight.pipelineSwitch(1);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            LLStatus status = limelight.getStatus();
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

        }
        limelight.stop();
    }

}
