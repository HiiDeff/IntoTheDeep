package org.firstinspires.ftc.teamcode.drive.V5Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.V5IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.V5IntakeClawTask;
import org.firstinspires.ftc.teamcode.task.V5IntakeWristTask;
import org.firstinspires.ftc.teamcode.util.Utils;

@Config
public class V5Robot extends MecanumDrive {

    public DcMotorEx horizontalSlides;

    public Servo claw;
    public Servo intakeArm;
    public Servo intakeWrist;
    public Limelight3A limelight;

    // Intake Servo Positions
    public static double INTAKE_ARM_DOWN = 0,
            INTAKE_ARM_TRANSFER = 1.0;
    // Claw Servo Positions
    public static double CLAW_OPEN_POS = 0.3, //0.35,
            CLAW_CLOSE_POS = 0.0;
    //Rotate Claw Servo Positions
    public static double ROTATECLAW_STRAIGHT_POS = 0.11,
            ROTATECLAW_LEFT_POS = 0.14,
            ROTATECLAW_RIGHT_POS = 0.09,
            ROTATECLAW_SIDE_POS = 0.05;

    public static double HORIZONTAL_EXTEND_MIN_IN = -0.05, HORIZONTAL_EXTEND_MAX_IN = 22.0;
    public static double HORIZONTAL_TICKS_PER_INCH = 31.8181818;

    private int horizontalSlidesPos = 0;
    private int lastHorizontalSlidesPos = 0;

    private void setDriveParams() {

        PinpointDrive.Params pinpointParams = PinpointDrive.PARAMS;
        pinpointParams.xOffset = -3.4;
        pinpointParams.yOffset = 1.5748;

        pinpointParams.encoderResolution = GoBildaPinpointDriverRR.goBILDA_4_BAR_POD;

        pinpointParams.xDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        pinpointParams.yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        MecanumDrive.Params p = MecanumDrive.PARAMS;
        p.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        p.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        // drive model parameters
        p.inPerTick = 1.0;
        p.lateralInPerTick = 0.6278478255385573; // p.inPerTick;
        p.trackWidthTicks = 14.230488919269296;

        // feedforward parameters (in tick units)
        p.kS = 1.2466769812779628;
        p.kV = 0.13364281383024199;
        p.kA = 0.03;

        // path profile parameters (in inches)
        p.maxWheelVel = 60;
        p.minProfileAccel = -30; //TODO change this to -40
        p.maxProfileAccel = 50; //TODO change this to 60

        // turn profile parameters (in radians)
        p.maxAngVel = Math.PI; // shared with path
        p.maxAngAccel = Math.PI;

        // path controller gains
        p.axialGain = 5.0;
        p.lateralGain = 7.0;
        p.headingGain = 4.0; // shared with turn

        p.axialVelGain = 0.0;
        p.lateralVelGain = 0.0;
        p.headingVelGain = 0.0; // shared with turn

    }

    public V5Robot(HardwareMap hardwareMap){
        super(new Pose2d(0.0, 0.0, 0.0));

        claw = hardwareMap.get(Servo.class, "claw");
        intakeArm = hardwareMap.get(Servo.class, "intakeArm");
        intakeWrist = hardwareMap.get(Servo.class, "rotateClawServo");
        horizontalSlides = hardwareMap.get(DcMotorEx.class, "horizontalSlides");
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        horizontalSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setDriveParams();
        init(hardwareMap);
    }

    public void teleOpInit() {
        setIntakeArmPos(V5IntakeArmTask.Position.DOWN);
        setClawPosition(V5IntakeClawTask.ClawPosition.OPEN);
    }

    public void updateEverything() {
        updateEncoders();
        //updatePoseEstimate();
    }

    public void setClawPosition(V5IntakeClawTask.ClawPosition position){
        claw.setPosition(getClawPosition(position));
    }

    public void setIntakeArmPos(V5IntakeArmTask.Position position){
        intakeArm.setPosition(getIntakeArmPosition(position));
    }

    public void setWristPos(V5IntakeWristTask.WristPosition position){
        intakeWrist.setPosition(getWristPosition(position));
    }

    public double getIntakeArmPosition(V5IntakeArmTask.Position position) {
        switch(position) {
            case TRANSFER:
                return INTAKE_ARM_TRANSFER;
            case DOWN:
            default:
                return INTAKE_ARM_DOWN;
        }
    }

    public double getClawPosition(V5IntakeClawTask.ClawPosition position) {
        switch(position) {
            case CLOSE:
                return CLAW_CLOSE_POS;
            case OPEN:
            default:
                return CLAW_OPEN_POS;
        }
    }

    public double getWristPosition(V5IntakeWristTask.WristPosition position){
        switch (position){
            case VERTICAL:
                return ROTATECLAW_STRAIGHT_POS;
            case HORIZONTAL:
            default:
                return ROTATECLAW_SIDE_POS;
        }
    }

    public void resetHorizontalAndVerticalSlidesEncoderPositions() {
        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getHorizontalSlideDistInch() {
        return ((double) horizontalSlides.getCurrentPosition()) / HORIZONTAL_TICKS_PER_INCH;
    }

    public void updateLastHorizontalEncoder() {
        lastHorizontalSlidesPos = horizontalSlidesPos;
    }

    public void setHorizontalSlidesPowerWithoutEncoder(double power) {
        if(horizontalSlides.getMode() != DcMotorEx.RunMode.RUN_WITHOUT_ENCODER) {
            horizontalSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        horizontalSlides.setPower(power);
    }

    public void brakeHorizontalSlides(double brakePower) {
        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalSlides.setTargetPosition(lastHorizontalSlidesPos);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlides.setPower(brakePower);
    }

    public void setHorizontalSlidesPower(double power) {
        if(horizontalSlides.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
            horizontalSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        horizontalSlides.setPower(power);
    }
    public void setHorizontalSlideDist(double posInches, double power) {
        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalSlides.setTargetPosition((int) (
                (Utils.clamp(posInches, HORIZONTAL_EXTEND_MIN_IN, HORIZONTAL_EXTEND_MAX_IN)
                        * HORIZONTAL_TICKS_PER_INCH)));
        horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlides.setPower(power);
    }
    public void updateEncoders(){
        horizontalSlidesPos = horizontalSlides.getCurrentPosition();
    }
}

