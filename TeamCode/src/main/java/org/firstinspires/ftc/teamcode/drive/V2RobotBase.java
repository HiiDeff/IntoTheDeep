package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.task.ClawTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.V2DiffyTask;
import org.firstinspires.ftc.teamcode.util.Utils;

@Config
public abstract class V2RobotBase extends MecanumDrive {


    // Slides Constants
    //TODO
    public static double VERTICAL_EXTEND_MIN_IN = -0.2;
    public static double VERTICAL_EXTEND_MAX_IN = 39;
    public static double VERTICAL_TICKS_PER_INCH = 32.91 * 14 / 10; // 14 / 10 gear ratio

    public static double HORIZONTAL_EXTEND_MIN_IN = -0.05;
    public static double HORIZONTAL_EXTEND_MAX_IN = 19.0;
    public static double HORIZONTAL_TICKS_PER_INCH = 32.91;
    public static double HORIZONTAL_SLIDE_TRANSFER_POS = 147/HORIZONTAL_TICKS_PER_INCH;

    // States

    private boolean intakeArmUp; //horizontalSlideExtended,

    // Common
    protected final HardwareMap hardwareMap;

    // Motors
    public final DcMotorEx verticalSlides;
    public final DcMotorEx horizontalSlides;
    public final DcMotorEx intakeRoller;


    // Servos
    public final Servo diffyLeft;
    public final Servo diffyRight;
    public final Servo outtakeClaw;
    public final Servo intakeArm;

    // Cached Encoder Vals
    private int verticalSlidesPos = 0;
    private int horizontalSlidesPos = 0;


    public V2RobotBase(HardwareMap hardwareMap) {
        super(new Pose2d(0.0, 0.0, 0.0));
        this.hardwareMap = hardwareMap;
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //TODO
        //reverse motors
        // Motors:
        verticalSlides = hardwareMap.get(DcMotorEx.class, "verticalSlides");
        verticalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        horizontalSlides = hardwareMap.get(DcMotorEx.class, "horizontalSlides");
        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeRoller = hardwareMap.get(DcMotorEx.class, "intakeRoller");
        intakeRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servos:
        diffyLeft = hardwareMap.get(Servo.class, "diffyLeft");
        diffyRight = hardwareMap.get(Servo.class, "diffyRight");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        intakeArm = hardwareMap.get(Servo.class, "intakeArm");
        intakeArmUp = false;
    }

    public void teleOpInit() {
        setIntakeArmPos(V2Robot.INTAKE_ARM_DOWN); //very bad coding convention, fix later
       // setHorizontalSlideDist(HORIZONTAL_SLIDE_TRANSFER_POS, 0.8);
        setDiffyPosition(V2DiffyTask.DiffyPosition.CARRY);
    }
    public void autoInit() {

    }
    public void updateEverything() {
        updateEncoders();
        //updatePoseEstimate();
    }

    public void updateEncoders() {
        verticalSlidesPos = verticalSlides.getCurrentPosition();
        horizontalSlidesPos = horizontalSlides.getCurrentPosition();
    }

//    public boolean verticalSlidesBound() {
//        if(Math.min(verticalSlidesLeftPos, verticalSlidesRightPos)*VERTICAL_TICKS_PER_INCH <= VERTICAL_EXTEND_MIN_IN){
//            //setVerticalSlidesHeight(VERTICAL_EXTEND_MIN_IN*VERTICAL_TICKS_PER_INCH, 0.8);
//            return true;
//        }
//        if(Math.max(verticalSlidesLeftPos, verticalSlidesRightPos)*VERTICAL_TICKS_PER_INCH >= VERTICAL_EXTEND_MAX_IN) {
//            //setVerticalSlidesHeight(VERTICAL_EXTEND_MAX_IN*VERTICAL_TICKS_PER_INCH, 0.8);
//            return true;
//        }
//        else return false;
//    }

    public void setVerticalSlidePower(double power) {
        verticalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlides.setPower(Utils.clamp(power, -1.0, 1.0));
    }// Utils for Vertical Slides
    public void setVerticalSlidePower(double power, double standPower) {
        verticalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(verticalSlidesPos > VERTICAL_EXTEND_MAX_IN*VERTICAL_TICKS_PER_INCH) {
            verticalSlides.setPower(Utils.clamp(standPower, -1.0, 1.0));
        }
        else {
            verticalSlides.setPower(Utils.clamp(power, -1.0, 1.0));
        }
    }
    public void setDefaultVerticalSlidesHeight(double power) {
        verticalSlides.setTargetPosition((int) (
                (Utils.clamp(VERTICAL_EXTEND_MAX_IN, VERTICAL_EXTEND_MIN_IN, VERTICAL_EXTEND_MAX_IN)
                        * VERTICAL_TICKS_PER_INCH)));
        verticalSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        verticalSlides.setPower(power);
    }
    public void setVerticalSlidesHeight(double posInches, double power) {
        verticalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlides.setTargetPosition((int) (
                (Utils.clamp(posInches, VERTICAL_EXTEND_MIN_IN, VERTICAL_EXTEND_MAX_IN)
                        * VERTICAL_TICKS_PER_INCH)));
        verticalSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        verticalSlides.setPower(power);
    }

    public double getVerticalSlidesHeight()
    {
        int ticks = verticalSlides.getCurrentPosition();
        return ((double)ticks)/VERTICAL_TICKS_PER_INCH;
    }

    // Utils for Horizontal Slides

//    public void toggleHorizontalSlides() {
//        if(!horizontalSlideExtended) {
//            setHorizontalSlideDist(HORIZONTAL_EXTEND_MAX_IN, 0.8);
//        }else{
//            setHorizontalSlideDist(HORIZONTAL_SLIDE_TRANSFER_POS, 0.8);
//        }
//        horizontalSlideExtended = !horizontalSlideExtended;
//    }

    public void setHorizontalSlideDist(double posInches) {
        horizontalSlides.setTargetPosition((int) (
                (Utils.clamp(posInches, HORIZONTAL_EXTEND_MIN_IN, HORIZONTAL_EXTEND_MAX_IN)
                        * HORIZONTAL_TICKS_PER_INCH)));
        horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setHorizontalSlideDist(double posInches, double power) {
        horizontalSlides.setTargetPosition((int) (
                (Utils.clamp(posInches, HORIZONTAL_EXTEND_MIN_IN, HORIZONTAL_EXTEND_MAX_IN)
                        * HORIZONTAL_TICKS_PER_INCH)));
        horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlides.setPower(power);
    }

    public void setHorizontalSlidesPower(double power) {
        if(horizontalSlides.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
            horizontalSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        horizontalSlides.setPower(power);
    }

    public double getHorizontalSlideDist() {
        int ticks = horizontalSlides.getCurrentPosition();
        return ((double)ticks)/HORIZONTAL_TICKS_PER_INCH;
    }

    // Utils for Diffy Arm

    public abstract double getDiffyTargetPos(V2DiffyTask.WhichDiffy servo, V2DiffyTask.DiffyPosition position);

    public void setDiffyPosition(V2DiffyTask.DiffyPosition diffyPosition) {
        V2DiffyTask.setDiffyPosition(this, V2DiffyTask.WhichDiffy.LEFT, diffyPosition);
        V2DiffyTask.setDiffyPosition(this, V2DiffyTask.WhichDiffy.RIGHT, diffyPosition);
    }


    // Temporary Solution
    //TODO: make tasks and stop being lazy


    // Utils for Intake Arm
    public void toggleIntakeArm() {
        if(!intakeArmUp) {
            setIntakeArmPos(getIntakeArmPos(IntakeArmTask.Position.TRANSFER));
        }else{
            setIntakeArmPos(getIntakeArmPos(IntakeArmTask.Position.DOWN));
        }
        intakeArmUp = !intakeArmUp;
    }
    public abstract double getIntakeArmPos(IntakeArmTask.Position position);
    public void setIntakeArmPos(double pos) {
        intakeArm.setPosition(Utils.clamp(pos, 0, 1));
    }

    // Utils for Roller Intake
    public void runIntakeIn() {
        intakeRoller.setPower(1.0);
    }
    public void runIntakeOut() {
        intakeRoller.setPower(-1.0);
    }
    public void stopIntake() {
        intakeRoller.setPower(0.0);
    }

    // Utils for claw

    public abstract double getClawPosition(ClawTask.ClawPosition position);
    public void setClawPosition(double pos) {
        outtakeClaw.setPosition(Utils.clamp(pos, 0.0, 1.0));
    }
}
