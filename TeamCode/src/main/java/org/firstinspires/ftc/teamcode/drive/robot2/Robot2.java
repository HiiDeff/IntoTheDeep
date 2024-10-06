package org.firstinspires.ftc.teamcode.drive.robot2;//package org.firstinspires.ftc.teamcode.drive.robot2;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.drive.WorldRobotBase;
//import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
//import org.firstinspires.ftc.teamcode.task.DeliveryArmTask;
//import org.firstinspires.ftc.teamcode.task.DeliveryBoxTask;
//import org.firstinspires.ftc.teamcode.task.DeliveryStopperTask;
//import org.firstinspires.ftc.teamcode.task.HangTask;
//import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
//
//@Config
//public class Robot2 extends WorldRobotBase {
//
//    // Intake Constants
//    public static double INTAKE_ARM_UP = 0.3; //old 0.2
//    public static double INTAKE_ARM_FULL_STACK = 0.44; //old 0.37
//    public static double INTAKE_ARM_4_PIXELS = 0.47; //old 0.385
//    public static double INTAKE_ARM_3_PIXELS = 0.50; //old 0.43
//    public static double INTAKE_ARM_2_PIXELS = 0.52; //old 0.48
//    public static double INTAKE_ARM_DOWN = 0.56; //old 0.48
//    public static double INTAKE_ARM_TOUCH = 0.50;
//
//    // Delivery stopper
//    public static double DELIVERY_STOPPER_CLOSED = 0.31, DELIVERY_DROP_ONE = 0.55,
//            DELIVERY_DROP_BOTH = 0.41;
//
//    // Delivery box
//    public static double DELIVERY_BOX_INTAKE = 0.2, DELIVERY_BOX_DELIVER = 0.55,
//            DELIVERY_BOX_REARRANGE = 0.9;
//
//    // Deliver arm
//    public static double DELIVERY_ARM_INTAKE = 0.51, DELIVERY_ARM_DELIVER = 0.0,
//            DELIVERY_ARM_REARRANGE = 0.0;
//
//    // Hang
//    public static double LEFT_INIT_POS = 0.3;
//    public static double LEFT_RELEASE_POS = 0.4;
//    public static double LEFT_HANG_POS = 0.8;
//    public static double RIGHT_INIT_POS = 0.7;
//    public static double RIGHT_RELEASE_POS = 0.6;
//    public static double RIGHT_HANG_POS = 0.2;
//
//    public Robot2(HardwareMap hardwareMap) {
//        super(hardwareMap);
//        setDriveParams();
//        init(hardwareMap, LocalizerMode.TWO_DEAD_WHEEL);
//    }
//
//    private void setDriveParams() {
//        Params p = MecanumDrive.PARAMS;
//        p.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
//        p.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
//
//        // drive model parameters
//        p.inPerTick = 0.002955;
//        p.lateralInPerTick = 0.0020686; // p.inPerTick;
//        p.trackWidthTicks = 4438;
//
//        // feedforward parameters (in tick units)
//        p.kS = 0.85203712;
//        p.kV = 0.000433642;
//        p.kA = 0.00008;
//
//        // path profile parameters (in inches)
//        p.maxWheelVel = 60;
//        p.minProfileAccel = -40;
//        p.maxProfileAccel = 60;
//
//        // turn profile parameters (in radians)
//        p.maxAngVel = Math.PI; // shared with path
//        p.maxAngAccel = Math.PI;
//
//        // path controller gains
//        p.axialGain = 5.0;
//        p.lateralGain = 5.0;
//        p.headingGain = 3.0; // shared with turn
//
//        p.axialVelGain = 0.0;
//        p.lateralVelGain = 0.0;
//        p.headingVelGain = 0.0; // shared with turn
//
//        TwoDeadWheelLocalizer.Params localizerParams = TwoDeadWheelLocalizer.PARAMS;
//        localizerParams.perpXTicks = -0.375;
//        localizerParams.parYTicks = 0.0;
//    }
//
//    @Override
//    public double getIntakeArmPos(IntakeArmTask.Position pos) {
//        switch (pos) {
//            case DOWN:
//                return INTAKE_ARM_DOWN;
//            case PIXEL_2_HEIGHT:
//                return INTAKE_ARM_2_PIXELS;
//            case PIXEL_3_HEIGHT:
//                return INTAKE_ARM_3_PIXELS;
//            case PIXEL_4_HEIGHT:
//                return INTAKE_ARM_4_PIXELS;
//            case FULL_STACK:
//                return INTAKE_ARM_FULL_STACK;
//            case UP:
//            default:
//                return INTAKE_ARM_UP;
//        }
//    }
//
//    @Override
//    public double getDeliveryStopperPos(DeliveryStopperTask.Position pos) {
//        switch (pos) {
//            case CLOSED:
//                return DELIVERY_STOPPER_CLOSED;
//            case DROP_ONE:
//                return DELIVERY_DROP_ONE;
//            case DROP_BOTH:
//            case OPEN:
//            default:
//                return DELIVERY_DROP_BOTH;
//        }
//    }
//
//    @Override
//    public double getDeliveryBoxPos(DeliveryBoxTask.Position pos) {
//        switch (pos) {
//            case INTAKE:
//                return DELIVERY_BOX_INTAKE;
//            case DELIVERY:
//                return DELIVERY_BOX_DELIVER;
//            case REARRANGE:
//            default:
//                return DELIVERY_BOX_REARRANGE;
//        }
//    }
//
//    @Override
//    public double getDeliveryArmPos(DeliveryArmTask.Position pos) {
//        switch (pos) {
//            case INTAKE:
//                return DELIVERY_ARM_INTAKE;
//            case DELIVERY:
//                return DELIVERY_ARM_DELIVER;
//            case REARRANGE:
//            default:
//                return DELIVERY_ARM_REARRANGE;
//        }
//    }
//    @Override
//    public double getHangTargetPos(HangTask.WhichHang placer, HangTask.Position position) {
//        switch (position) {
//            case INIT:
//                return placer == HangTask.WhichHang.LEFT ? LEFT_INIT_POS: RIGHT_INIT_POS;
//            case RELEASE:
//                return placer == HangTask.WhichHang.LEFT ? LEFT_RELEASE_POS : RIGHT_RELEASE_POS;
//            case HANG:
//            default:
//                return placer == HangTask.WhichHang.LEFT ? LEFT_HANG_POS : RIGHT_HANG_POS;
//        }
//    }
//}