package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.LM1Robot;
import org.firstinspires.ftc.teamcode.util.GamePad;

//@Config
@TeleOp(name = "LM1 TeleOp")

public class LM1TeleOp extends LinearOpMode {

    private static final double MIN_DRIVE_POW = 0.001;

    private static final double DELIVERY_STAND_POWER = 0.1;

    private LM1Robot robot;
    private GamePad gp1, gp2;
    private ElapsedTime intakeTimer;
    private boolean lastDelivery;

    // test stuff:
    public static double MAX_POS_DRIVE = 5, MAX_NEG_DRIVE = -100;
    public static double MAX_POS_TURN = 100, MAX_NEG_TURN = -100;
//    private SlewRateLimiter driveLim = new SlewRateLimiter(MAX_POS_DRIVE, MAX_NEG_DRIVE, 0.0);
//    private SlewRateLimiter turnLim = new SlewRateLimiter(MAX_POS_TURN, MAX_NEG_TURN, 0.0);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new LM1Robot(hardwareMap);
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
            gp1.update();
            gp2.update();
            telemetry.addData("cycle time", cycleTimer.milliseconds() - lastTime);
            telemetry.addData("pose (may be inaccurate)", robot.pose);
            lastTime = cycleTimer.milliseconds();
            telemetry.update();
            robot.updateEverything();
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

    private void runIntake() {

    }

    private void runDelivery() {

    }
}
