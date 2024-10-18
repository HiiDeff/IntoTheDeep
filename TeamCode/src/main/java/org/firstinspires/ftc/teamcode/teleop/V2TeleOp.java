package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.V2RobotBase;
import org.firstinspires.ftc.teamcode.util.GamePad;

//@Config
@TeleOp(name = "V2 TeleOp")
@Config

public class V2TeleOp extends LinearOpMode {

    public static double VERTICAL_STAND_POWER = 0.001; //chng to private
    public static double VERTICAL_SLIDE_MAX_POWER = 0.5;
    public static double MIN_DRIVE_POWER = 0.01;

    private V2RobotBase robot;
    private GamePad gp1, gp2;
    private ElapsedTime intakeTimer;
    private boolean lastDelivery;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = (V2RobotBase) RobotFactory.createRobot(hardwareMap);
        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);

        robot.teleOpInit();
        waitForStart();
        ElapsedTime cycleTimer = new ElapsedTime();
        double lastTime = cycleTimer.milliseconds();
        while (opModeIsActive()) {
            //driveRobot();
            runIntake();
            runDelivery();
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


    private void runDelivery() {
        if(gp2.onceX()) { //delivery slides up and diffy carry
            robot.setDefaultVerticalSlidesHeight(VERTICAL_SLIDE_MAX_POWER);
            robot.setDiffyPosition(V2RobotBase.DiffyPosition.CARRY); //ALSO BAD CONVENTION TODO: FIX
        }
        else if(gp2.onceY()) { //diffy transfer
            robot.setDiffyPosition(V2RobotBase.DiffyPosition.TRANSFER);
        }
        else if(gp2.onceA()) { //diffy deliver
            robot.setDiffyPosition(V2RobotBase.DiffyPosition.DELIVER);
        }



        if(Math.abs(gp2.leftStickY()) > MIN_DRIVE_POWER){
            robot.setVerticalSlidePower(-gp2.leftStickY()*VERTICAL_SLIDE_MAX_POWER, VERTICAL_STAND_POWER);
        }
        else{
            robot.setVerticalSlidePower(VERTICAL_STAND_POWER);
        }
    }

    private void runIntake() {
        if(gp2.onceLeftBumper()){
            robot.toggleHorizontalSlides();
        }
        if(gp2.onceRightBumper()){
            robot.toggleIntakeArm();
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
