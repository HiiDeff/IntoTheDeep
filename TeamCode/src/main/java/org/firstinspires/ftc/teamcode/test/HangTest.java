package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.IntakeSampleState;
import org.firstinspires.ftc.teamcode.util.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.util.profile.ServoProfiler;

@TeleOp(name = "Hang Test", group = "Test")
@Config

public class HangTest extends LinearOpMode {


    public MultipleTelemetry multipleTelemetry;
    // Servos
    public Servo leftHang, rightHang;
    public static double leftHangPos, rightHangPos;
    public static boolean testLeftHang, testRightHang;

    @Override
    public void runOpMode() throws InterruptedException {

        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        leftHang = hardwareMap.get(Servo.class, "leftHang");
        rightHang = hardwareMap.get(Servo.class, "rightHang");

        waitForStart();
        while(opModeIsActive()) {

            if(testLeftHang) leftHang.setPosition(leftHangPos);
            if(testRightHang) rightHang.setPosition(rightHangPos);

        }

    }
}
