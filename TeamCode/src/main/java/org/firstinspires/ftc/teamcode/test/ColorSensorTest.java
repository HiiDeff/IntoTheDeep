package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.V3Robot.V3Robot;
import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "Color Sensor Tester", group = "Test")
@Disabled
@Config

public class ColorSensorTest extends LinearOpMode {

    private GamePad gp1;

    private RevColorSensorV3 leftIntakeColorSensor, rightIntakeColorSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        leftIntakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColorLeft");
        rightIntakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColorRight");


        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("left dist", leftIntakeColorSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("right dist", rightIntakeColorSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();

        }
    }
}
