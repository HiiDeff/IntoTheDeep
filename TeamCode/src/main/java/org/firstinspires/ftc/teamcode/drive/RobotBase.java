package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.task.DiffyIntakeTask;
import org.firstinspires.ftc.teamcode.util.Utils;

@Config
public abstract class RobotBase extends MecanumDrive {


    // Slides Constants
    public static double VERTICAL_EXTEND_MIN_IN = 2;
    public static double VERTICAL_EXTEND_MAX_IN = 37.5; //TODO
    public static double VERTICAL_TICKS_PER_INCH = 4391.5/38.189; //TODO

    public static double HORIZONTAL_EXTEND_MIN_IN = -0.2;
    public static double HORIZONTAL_EXTEND_MAX_IN = 0.0; //TODO
    public static double HORIZONTAL_TICKS_PER_INCH = 0.0; //TODO

    // States

    private int intakeRollerDirection;

    // Common
    protected final HardwareMap hardwareMap;

    // Motors
    public final DcMotorEx verticalSlidesLeft;
    public final DcMotorEx verticalSlidesRight;
    //public final DcMotorEx horizontalSlides;

    // Servos
    public final Servo diffyLeft;
    public final Servo diffyRight;
    public final Servo intakeRoller;

    // Cached Encoder Vals
    private int verticalSlidesLeftPos = 0;
    private int verticalSlidesRightPos = 0;
    private int horizontalSlidesPos = 0;


    public RobotBase(HardwareMap hardwareMap) {
        super(new Pose2d(0.0, 0.0, 0.0));
        this.hardwareMap = hardwareMap;
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        // Motors:
        verticalSlidesLeft = hardwareMap.get(DcMotorEx.class, "verticalSlidesLeft");
        verticalSlidesLeft.setDirection(DcMotorEx.Direction.REVERSE);
        verticalSlidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalSlidesRight = hardwareMap.get(DcMotorEx.class, "verticalSlidesRight");
        verticalSlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        horizontalSlides = hardwareMap.get(DcMotorEx.class, "horizontalSlides");
//        horizontalSlides.setDirection(DcMotorEx.Direction.REVERSE);
//        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Servos:
        diffyLeft = hardwareMap.get(Servo.class, "diffyLeft");
        diffyRight = hardwareMap.get(Servo.class, "diffyRight");
        intakeRoller = hardwareMap.get(Servo.class, "intakeRoller");
        intakeRollerDirection = 0;

    }

    public void teleOpInit() {

        setVerticalSlidesHeight(VERTICAL_EXTEND_MIN_IN, 0.8);
        setDiffyPosition(LM1Robot.INTAKE_CENTER);
    }
    public void autoInit() {

    }
    public void updateEverything() {
        updateEncoders();
        //updatePoseEstimate();
    }

    public void updateEncoders() {
        verticalSlidesLeftPos = verticalSlidesLeft.getCurrentPosition();
        verticalSlidesRightPos = verticalSlidesRight.getCurrentPosition();
        //horizontalSlidesPos = horizontalSlides.getCurrentPosition();
    }

    public boolean verticalSlidesBound() {
        if(Math.min(verticalSlidesLeftPos, verticalSlidesRightPos)*VERTICAL_TICKS_PER_INCH <= VERTICAL_EXTEND_MIN_IN){
            //setVerticalSlidesHeight(VERTICAL_EXTEND_MIN_IN*VERTICAL_TICKS_PER_INCH, 0.8);
            return true;
        }
        if(Math.max(verticalSlidesLeftPos, verticalSlidesRightPos)*VERTICAL_TICKS_PER_INCH >= VERTICAL_EXTEND_MAX_IN) {
            //setVerticalSlidesHeight(VERTICAL_EXTEND_MAX_IN*VERTICAL_TICKS_PER_INCH, 0.8);
            return true;
        }
        else return false;
    }
    // Utils for Vertical Slides
    public void setVerticalSlidePower(double power) {
        verticalSlidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlidesLeft.setPower(Utils.clamp(power, -1.0, 1.0));
        verticalSlidesRight.setPower(Utils.clamp(power, -1.0, 1.0));
    }
    public void setVerticalSlidesHeight(double posInches, double power) {
        verticalSlidesLeft.setTargetPosition((int) (
                (Utils.clamp(posInches, VERTICAL_EXTEND_MIN_IN, VERTICAL_EXTEND_MAX_IN)
                        * VERTICAL_TICKS_PER_INCH)));
        verticalSlidesRight.setTargetPosition((int) (
                (Utils.clamp(posInches, VERTICAL_EXTEND_MIN_IN, VERTICAL_EXTEND_MAX_IN)
                        * VERTICAL_TICKS_PER_INCH)));
        verticalSlidesLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        verticalSlidesRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        verticalSlidesLeft.setPower(power);
        verticalSlidesRight.setPower(power);
    }


    // Utils for Horizontal Slides

//    public void setHorizontalSlidePowerWithoutEncoder(double power) {
//        horizontalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        horizontalSlides.setPower(Utils.clamp(power, -1.0, 1.0));
//    }
//
//    public void setDeliverySlideDist(double posInches) {
//        horizontalSlides.setTargetPosition((int) (
//                (Utils.clamp(posInches, HORIZONTAL_EXTEND_MIN_IN, HORIZONTAL_EXTEND_MAX_IN)
//                        * HORIZONTAL_TICKS_PER_INCH)));
//        horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    public void setDeliverySlideDist(double posInches, double power) {
//        horizontalSlides.setTargetPosition((int) (
//                (Utils.clamp(posInches, HORIZONTAL_EXTEND_MIN_IN, HORIZONTAL_EXTEND_MAX_IN)
//                        * HORIZONTAL_TICKS_PER_INCH)));
//        horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        horizontalSlides.setPower(power);
//    }

    // Utils for Diffy Arm

    public abstract DiffyPosition setDiffyPosition(DiffyIntakeTask.DiffyPosition position);

    public void setDiffyPosition(LM1Robot.DiffyPosition diffyPosition) {
        diffyLeft.setPosition(diffyPosition.left);
        diffyRight.setPosition(diffyPosition.right);
    }

    public static class DiffyPosition {
        double left, right;
        public DiffyPosition(double a, double b){
            this.left = a;
            this.right = b;
        }
    }


    // Utils for Roller Intake
    public void runIntakeIn() {
        intakeRoller.setPosition(0.0);
//        if(intakeRollerDirection==-1 || intakeRollerDirection==0){
//            intakeRoller.setPosition(1);
//        }
//        else{
//            intakeRoller.setPosition(0);
//        }
//        intakeRollerDirection = 1;
    }
    public void runIntakeOut() {
        intakeRoller.setPosition(1.0);
//        if(intakeRollerDirection==1 || intakeRollerDirection==0){
//            intakeRoller.setPosition(-1);
//        }
//        else{
//            intakeRoller.setPosition(0);
//        }
//        intakeRollerDirection = -1;
    }
    public void stopIntake() {
        intakeRoller.setPosition(0.5);
    }
}
