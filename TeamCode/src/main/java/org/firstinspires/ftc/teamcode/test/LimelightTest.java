package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.limelight.Coordinates;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightConfig;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightProcessorBase;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightSampleDetector;
import org.firstinspires.ftc.teamcode.util.limelight.ObjectOrientation;

import java.util.List;

@TeleOp(name = "Limelight Tester", group = "Test")
@Config
public class LimelightTest extends LinearOpMode {

//    public static LimelightConfig LLConfig = new LimelightConfig(320, 240,
//            -30, 54,41,
//            7.5,-3.5,12.75); // ROBOT 3
    public static LimelightConfig LLConfig = new LimelightConfig(320, 240,
        -30, 54,41,
        7.7,4.0,13.75); //ROBOT 4 MEASUREMENTS
    private LimelightSampleDetector limelightSampleDetector;
    private Limelight3A limelight;
    public MultipleTelemetry multipleTelemetry;


    @Override
    public void runOpMode() throws InterruptedException {

        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelightSampleDetector = new LimelightSampleDetector(limelight, LLConfig);

        limelight.pipelineSwitch(1);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            LLStatus status = limelight.getStatus();
            multipleTelemetry.addData("Name", "%s",
                    status.getName());
            multipleTelemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            multipleTelemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            Coordinates objectPosition = limelightSampleDetector.getObjectPosition(0.5);
            multipleTelemetry.addData("objectXPos",objectPosition.x);
            multipleTelemetry.addData("objectYPos",objectPosition.y);

            ObjectOrientation orientation = limelightSampleDetector.getObjectOrientation();
            multipleTelemetry.addData("orientation",orientation.toString());

            multipleTelemetry.update();
            limelightSampleDetector.updateLimelight();

        }
        limelight.stop();
    }

}
