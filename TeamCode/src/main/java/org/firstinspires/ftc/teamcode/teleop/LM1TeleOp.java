package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.LM1Robot;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.util.GamePad;

//@Config
@TeleOp(name = "LM1 TeleOp")
@Config

public class LM1TeleOp extends LinearOpMode {

    public static double VERTICAL_STAND_POWER = 0.001; //chng to private
    public static double SLIDES_UP_POWER = 0.5;
    public static double SLIDES_DOWN_POWER = -0.5;

    private RobotBase robot;
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
        robot = (RobotBase) RobotFactory.createRobot(hardwareMap);
        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);

        robot.teleOpInit();
        waitForStart();
        ElapsedTime cycleTimer = new ElapsedTime();
        double lastTime = cycleTimer.milliseconds();
        while (opModeIsActive()) {
            driveRobot();
            runIntake();
            runSlides();
            update();

            telemetry.addData("cycle time", cycleTimer.milliseconds() - lastTime);
            telemetry.addData("pose (may be slightlyyyy inaccurate)", robot.pose);
            lastTime = cycleTimer.milliseconds();
        }
    }
    private void update() {
        gp1.update();
        gp2.update();
        telemetry.update();
        robot.updateEverything();
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
            a = -gp1.rightStickY() * 0.8;
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


    private void runSlides() {
        if(robot.verticalSlidesBound()) {
            robot.setVerticalSlidePower(VERTICAL_STAND_POWER);
            //robot.setVerticalSlidePower(VERTICAL_STAND_POWER);
        }
        if(gp2.dpadDown()) {
            robot.setVerticalSlidePower(SLIDES_DOWN_POWER);
        }
        else if(gp2.dpadUp()) {
            robot.setVerticalSlidePower(SLIDES_UP_POWER);
        }
        else{
            robot.setVerticalSlidePower(VERTICAL_STAND_POWER);
        }

//        if(Math.abs(gp2.rightStickY()) > MIN_DRIVE_POW) {
//            robot.setHorizontalSlidePowerWithoutEncoder(-gp2.rightStickY());
//        }
    }

    private void runIntake() {
        if(gp2.leftBumper() && gp2.rightBumper()){
            robot.setDiffyPosition(LM1Robot.INTAKE_CENTER);
        }else if(gp2.leftBumper()) {
            robot.setDiffyPosition(LM1Robot.INTAKE_LEFT);
        }else if(gp2.rightBumper()) {
            robot.setDiffyPosition(LM1Robot.INTAKE_RIGHT);
        }else if(gp2.onceA()){
            robot.setDiffyPosition(LM1Robot.INTAKE_UP);
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
