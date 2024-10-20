package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.task.TimedTask;
import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "Slide Motor Tester", group = "Test")
@Config
public class SlideMotorTest extends LinearOpMode {

    public class TestTimedMotorTask extends TimedTask {
        private DcMotorEx motor;
        private final int position;
        private final double power;
        private boolean started = false;
        public TestTimedMotorTask(DcMotorEx motor, int position, double power,
                                  int finishTimeMillis) {
            this.motor = motor;
            this.position = position;
            this.power = power;
            setFinishTimeMillis(finishTimeMillis);
        }

        @Override
        protected boolean performInternal() {
            if (!started) {
                started = true;
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setTargetPosition(this.position);
                motor.setPower(POWER);
            }
            int currPos = motor.getCurrentPosition();
            multipleTelemetry.addData("currpos", currPos);
            multipleTelemetry.update();
            if (Math.abs(currPos - position) < 10) {
                cancel();
                return true;
            }
            return false;
        }

        @Override
        public void cancel() {
            motor.setPower(0.0);
        }

    }

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public MultipleTelemetry multipleTelemetry;
    DcMotorEx motor;
    GamePad gp1;
    public static double LengthInInches = 1;
    public static double POWER = 0.2;

    @Override
    public void runOpMode() {
        multipleTelemetry = new MultipleTelemetry(telemetry,
                dashboard.getTelemetry());

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "testMotor");
        TimedTask motorTask = null;

        gp1 = new GamePad(gamepad1);

        int runToPos = 0;

        waitForStart();
        telemetry.addData("started", 0);
        telemetry.update();
        while (opModeIsActive()) {
            if (motorTask != null)
            {
                if (motorTask.perform()) {
                    motorTask = null;
                }
            }
            else if (gp1.onceB()) {
                telemetry.addData("clicked", 1);
                telemetry.update();

                runToPos = (int) (LengthInInches * 32.91);
                multipleTelemetry.addData("run to pos", runToPos);
                multipleTelemetry.update();
                motorTask = new TestTimedMotorTask(motor, runToPos, POWER, 5000);
            }
            gp1.update();

        }
    }
}
