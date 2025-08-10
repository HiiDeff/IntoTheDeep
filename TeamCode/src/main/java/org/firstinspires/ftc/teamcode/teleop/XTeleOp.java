//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.drive.RobotFactory;
//import org.firstinspires.ftc.teamcode.drive.V308Robot.V308Robot;
//import org.firstinspires.ftc.teamcode.task.ClawTask;
//import org.firstinspires.ftc.teamcode.task.DeliveryArmTask;
//import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
//import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
//import org.firstinspires.ftc.teamcode.task.ParallelTask;
//import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
//import org.firstinspires.ftc.teamcode.task.SeriesTask;
//import org.firstinspires.ftc.teamcode.task.SleepTask;
//import org.firstinspires.ftc.teamcode.task.SwipeStickTask;
//import org.firstinspires.ftc.teamcode.task.Task;
//import org.firstinspires.ftc.teamcode.task.TimedHorizontalSlidesTask;
////import org.firstinspires.ftc.teamcode.task.UnboundedIntakeTask;
//import org.firstinspires.ftc.teamcode.task.VerticalSlidesTask;
//import org.firstinspires.ftc.teamcode.util.GamePad;
//import org.firstinspires.ftc.teamcode.util.XGamepad;
//
//@TeleOp(name = "Shikanoko TeleOp [DO NOT USE]")
//@Config
//public class XTeleOp extends LinearOpMode {
//
//    private enum XTeleOpStatus {
//            STANDBY, SUB_INTAKE, TRANSFERRING, SPEC_DELIVER, SAMPLE_DELIVER
//    }
//
//    public static int CLAW_OPEN_DELAY = 330;
//    public static double CYCLE_DELIVER_X = 31.45, CYCLE_DELIVER_Y = 40.27, Y_OFFSET = 2.5;
//    public static double VERTICAL_HEIGHT_EPSILON = 1.0;
//    public static double VERTICAL_STAND_POWER = 0.2;
//    public static double MAX_HORIZONTAL_SLIDE_EXTENSION_INCH = 21.0;
//    public static double HORIZONTAL_SLIDE_POWER = 0.8;
//    public static double MIN_DRIVE_POWER = 0.01;
//    public static double DELIVERY_CHAMBER_HEIGHT = 21.3;
//    public static double MIN_HEIGHT_CONSIDERED_CHAMBER_DELIVERY = 15;
//    public static double DELIVERY_HIGH_BASKET_HEIGHT = 38.6;
//    public static double DELIVERY_LOW_BASKET_HEIGHT = 24.0;
//    private V308Robot robot;
//
//    public MultipleTelemetry multipleTelemetry;
//    private XGamepad gp1, gp2;
//    private boolean clawOpen, horizontalPositionUpdated, hangUp;
//    private Pose2d deliveryPose;
//    private Task presetTask;
//    private XTeleOpStatus status = XTeleOpStatus.STANDBY;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot = (V308Robot) RobotFactory.createRobot(hardwareMap);
//        robot.teleOpInit();
//        SwipeStickTask.setPosition(robot, SwipeStickTask.Position.IN);
//        gp1 = new XGamepad(new GamePad(gamepad1));
//        gp2 = new XGamepad(new GamePad(gamepad2));
//        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//        waitForStart();
//        while (opModeIsActive()) {
//            driveRobot();
//            if (status != XTeleOpStatus.SPEC_DELIVER) {
//                runSwipeStick();
//                if (status != XTeleOpStatus.TRANSFERRING) runIntake();
//                runHorizontalSlides();
//                runDelivery();
//            }
//            if (status == XTeleOpStatus.STANDBY) checkPreloads();
//            update();
//        }
//    }
//
//    private void update() {
//        if (presetTask != null && presetTask.perform()) {
//            status = XTeleOpStatus.STANDBY;
//            presetTask = null;
//        }
//        gp1.update();
//        gp2.update();
//        updateTelemetry();
//        robot.updateEverything();
//    }
//
//    private void updateTelemetry() {
//        telemetry.addData("colors", robot.getIntakeColorToString());
//        telemetry.addData("pose", robot.pose);
//        telemetry.addData("vertical slide position", robot.getVerticalSlidesHeightInch());
//        telemetry.addData("horizontal slide position", robot.getHorizontalSlideDistInch());
//        telemetry.update();
//    }
//
//    private void checkPreloads() {
//        if (gp2.sOnceA()) { // Submersible intake preset:
//            status = XTeleOpStatus.SUB_INTAKE;
//            presetTask = new SeriesTask(
//                    new ParallelTask(
//                            new IntakeArmTask(robot, IntakeArmTask.Position.CLEAR),
//                            new HorizontalSlidesTask(robot, 9.0, 1.0, 500)
//                    ),
//                    new IntakeArmTask(robot, IntakeArmTask.Position.DOWN)
//            );
//        } else if (gp2.sOnceB()) { // Transfer preset:
//            status = XTeleOpStatus.TRANSFERRING;
//            presetTask = new SeriesTask(
//                    new ParallelTask(
//                            new DeliveryArmTask(robot, DeliveryArmTask.Position.TRANSFER),
//                            new ClawTask(robot, ClawTask.ClawPosition.OPEN)
//                    ),
//                    new ParallelTask(
//                            new VerticalSlidesTask(robot, 0, 1.0, 2000),
//                            new SeriesTask(
//                                    new ParallelTask(
//                                            new TimedHorizontalSlidesTask(robot, robot.getHorizontalSlideTransferPos(), 1.0,1000),
//                                            new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER)
//                                            //new UnboundedIntakeTask(robot, 1.0)
//                                    ),
//                                    new ParallelTask(
//                                            new TimedHorizontalSlidesTask(robot, robot.getHorizontalSlideTransferPos(), 1.0, 500)
//                                            //new UnboundedIntakeTask(robot, -1.0)
//                                    ),
//                                    new ParallelTask(
//                                            new IntakeArmTask(robot, IntakeArmTask.Position.CLEAR),
//                                            new DeliveryArmTask(robot, DeliveryArmTask.Position.CARRY)
//                                    )
//                            )
//                    )
//            ); clawOpen = true;
//        } else if (gp1.sOnceSafeLeftTrigger()) { // Specimen delivery preset:
//            createSpecimenDeliveryTask(); clawOpen = true;
//        } else if (gp1.sOnceY()) { // Specimen delivery preset (no robot movement):
//            createSpecimenDeliveryTaskLite(); clawOpen = true;
//        } else if (gp2.aOnceY()) { // Basket delivery preset:
//            status = XTeleOpStatus.SAMPLE_DELIVER;
//            double height = gp2.back() ? DELIVERY_LOW_BASKET_HEIGHT : DELIVERY_HIGH_BASKET_HEIGHT;
//            presetTask = new ParallelTask(
//                    new VerticalSlidesTask(robot, height, 1.0, 1500),
//                    new DeliveryArmTask(robot, DeliveryArmTask.Position.CARRY)
//            );
//        }
//    }
//
//    private void driveRobot() {
//        double x = 0, y = 0, a = 0;
//        if (gp1.dpadLeft() || gp1.dpadRight()) {
//            a = 0.5 * (gp1.dpadLeft() ? 1 : -1);
//        } else if (gp1.dpadUp() || gp1.dpadDown()) {
//            x = 0.3 * (gp1.dpadUp() ? 1 : -1);
//        } else {
//            x = -gp1.leftStickY();
//            y = -gp1.leftStickX();
//            a = -gp1.rightStickX() * 0.9;
//        }
//        double pow = Math.sqrt(x * x + y * y);
//        if (status != XTeleOpStatus.SPEC_DELIVER) {
//            robot.setDrivePowers(new PoseVelocity2d(new Vector2d(x, y), a));
//        } else if (pow > MIN_DRIVE_POWER) {
//            status = XTeleOpStatus.STANDBY;
//            presetTask.cancel();
//            presetTask = null;
//        }
//    }
//
//    private void runSwipeStick() {
//        double mn = SwipeStickTask.IN_POSITION, mx = SwipeStickTask.OUT_POSITION;
//        double pos = mn + (mx - mn) * gp1.rightTrigger();
//        robot.swipeStick.setPosition(pos);
//    }
//
////    private void runIntake() {
////        boolean inputted = Math.max(Math.abs(gp2.rightStickY()), Math.abs(gp2.rightStickX()))
////                > gp2.getMinSafeJoystick() || gp2.rightStickButton() || gp2.safeLeftTrigger() ||
////                gp2.safeRightTrigger();
////        if (status == XTeleOpStatus.TRANSFERRING) {
////            if (inputted) {
////                status = XTeleOpStatus.STANDBY;
////                presetTask.cancel();
////                presetTask = null;
////            } else return;
////        }
////        if (gp2.intuitiveRightStickY() > gp2.getMinSafeJoystick()) {
////            robot.setIntakeArmPos(IntakeArmTask.Position.TRANSFER);
////        } else if (Math.abs(gp2.rightStickX()) > gp2.getMinSafeJoystick() || gp2.sOnceRightStickButton()) {
////            robot.setIntakeArmPos(IntakeArmTask.Position.CLEAR);
////        } else if (gp2.intuitiveLeftStickY() < -gp2.getMinSafeJoystick()) {
////            robot.setIntakeArmPos(IntakeArmTask.Position.DOWN);
////        }
////        if (gp2.safeRightTrigger()) {
////            robot.runIntakeIn();
////        } else if (gp2.safeLeftTrigger()) {
////            robot.runIntakeOut();
////        } else {
////            robot.stopIntake();
////        }
////    }
//
//    private void runHorizontalSlides() {
//        boolean inputted = Math.abs(gp2.leftStickY()) > gp2.getMinSafeJoystick() ||
//                gp2.leftStickButton();
//        if (status == XTeleOpStatus.TRANSFERRING || status == XTeleOpStatus.SUB_INTAKE) {
//            if (inputted) {
//                status = XTeleOpStatus.STANDBY;
//                presetTask.cancel();
//                presetTask = null;
//            } else return;
//        }
//        if (inputted && !(gp2.intuitiveLeftStickY() > 0 &&
//                robot.getHorizontalSlideDistInch() >= MAX_HORIZONTAL_SLIDE_EXTENSION_INCH)) {
//            robot.setHorizontalSlidesPowerWithoutEncoder(HORIZONTAL_SLIDE_POWER * (gp2.intuitiveLeftStickY() > 0 ? 1 : -1));
//            if (gp2.rightBumper()) {
//                robot.horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.horizontalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            } else {
//                robot.updateLastHorizontalEncoder();
//                horizontalPositionUpdated = true;
//            }
//        } else if (horizontalPositionUpdated) {
//            robot.brakeHorizontalSlides(0.05);
//            horizontalPositionUpdated = false;
//        }
//    }
//
//    private void runDelivery() {
//        boolean inputted = gp2.dpadUp() || gp2.dpadDown() || gp1.a() || gp1.b() || gp1.x() ||
//                gp1.onceComboBumpers();
//        if (status == XTeleOpStatus.TRANSFERRING || status == XTeleOpStatus.SAMPLE_DELIVER) {
//            if (inputted) {
//                status = XTeleOpStatus.STANDBY;
//                presetTask.cancel();
//                presetTask = null;
//            } else return;
//        }
//        if (gp2.dpadUp()) {
//            robot.setVerticalSlidePowerWithoutEncoder(1.0);
//        } else if (gp2.dpadDown()) {
//            robot.setVerticalSlidePowerWithoutEncoder(-0.3);
//        } else {
//            robot.setVerticalSlidePowerWithoutEncoder(robot.getVerticalSlidesHeightInch()
//                    < VERTICAL_HEIGHT_EPSILON ? 0 : VERTICAL_STAND_POWER);
//        }
//        if (gp1.sOnceX()) {
//            robot.setDeliveryArmServoPosition(DeliveryArmTask.Position.FLAT);
//            robot.setClawPosition(ClawTask.ClawPosition.OPEN); clawOpen = true;
//        } else if (gp1.sOnceB()) {
//            robot.setDeliveryArmServoPosition(DeliveryArmTask.Position.DELIVER);
//            robot.setClawPosition(ClawTask.ClawPosition.OPEN); clawOpen = true;
//        } else if (gp1.sOnceA()) {
//            robot.setDeliveryArmServoPosition(DeliveryArmTask.Position.TRANSFER);
//            robot.setClawPosition(ClawTask.ClawPosition.OPEN); clawOpen = true;
//        }
//        if (gp2.sOnceX()) {
//            robot.setClawPosition(clawOpen ? ClawTask.ClawPosition.CLOSE : ClawTask.ClawPosition.OPEN);
//            clawOpen = !clawOpen;
//        }
//        if (gp1.onceComboBumpers()) {
//            if (!hangUp) {
//                robot.releaseHang();
//            } else {
//                robot.lowerHang();
//                robot.setDeliveryArmServoPosition(DeliveryArmTask.Position.FLAT);
//            }
//            hangUp = !hangUp;
//        }
//    }
//
//    private void createSpecimenDeliveryTaskLite() {
//        status = XTeleOpStatus.SPEC_DELIVER;
//        if (robot.getVerticalSlidesHeightInch() > MIN_HEIGHT_CONSIDERED_CHAMBER_DELIVERY) {
//            presetTask = new ParallelTask(
//                    new HorizontalSlidesTask(robot, 0, 1.0, 700),
//                    new VerticalSlidesTask(robot, 0, 1.0, 1000),
//                    new SeriesTask(
//                            new SleepTask(1000),
//                            new DeliveryArmTask(robot, DeliveryArmTask.Position.FLAT)
//                    ),
//                    new SeriesTask(
//                            new SleepTask(CLAW_OPEN_DELAY),
//                            new ClawTask(robot, ClawTask.ClawPosition.OPEN)
//                    )
//            );
//        } else {
//            presetTask = new ParallelTask(
//                    new ClawTask(robot, ClawTask.ClawPosition.CLOSE),
//                    new ParallelTask(
//                            new SeriesTask(
//                                    new SleepTask(100),
//                                    new VerticalSlidesTask(robot, DELIVERY_CHAMBER_HEIGHT, 1.0, 2000)
//                            ),
//                            new SeriesTask(
//                                    new SleepTask(500),
//                                    new DeliveryArmTask(robot, DeliveryArmTask.Position.CHAMBER_DELIVER)
//                            )
//                    )
//            );
//        }
//    }
//
//    private void createSpecimenDeliveryTask() {
//        status = XTeleOpStatus.SPEC_DELIVER;
//        if (deliveryPose == null) {
//            robot.pinpoint.setPosition(new Pose2d(0, 0, 0.05));
//        }
//        SeriesTask task = new SeriesTask();
//        if (robot.getVerticalSlidesHeightInch() > MIN_HEIGHT_CONSIDERED_CHAMBER_DELIVERY) {
//            if (deliveryPose != null) {
//                deliveryPose = new Pose2d(robot.pinpoint.getPosition().getX(DistanceUnit.INCH),
//                        robot.pinpoint.getPosition().getY(DistanceUnit.INCH) + Y_OFFSET,
//                        Math.PI - 0.05);
//            }
//            task.add(
//                    new ParallelTask(
//                            new HorizontalSlidesTask(robot, 0, 1.0, 700),
//                            new VerticalSlidesTask(robot, 0, 1.0, 1000),
//                            new SeriesTask(
//                                    new SleepTask(1000),
//                                    new DeliveryArmTask(robot, DeliveryArmTask.Position.FLAT)
//                            ),
//                            new SeriesTask(
//                                    new SleepTask(CLAW_OPEN_DELAY),
//                                    new ClawTask(robot, ClawTask.ClawPosition.OPEN)
//                            ),
//                            new SeriesTask(
//                                    new SleepTask(100),
//                                    new RuntimeDrivingTask(
//                                            robot,
//                                            builder -> {
//                                                Pose2d pos = new Pose2d(-1, 0, 0.05);
//                                                return builder.strafeToLinearHeading(pos.position, pos.heading).build();
//                                            }
//                                    )
//                            )
//                    )
//            );
//        }
//        if (deliveryPose == null) {
//            deliveryPose = new Pose2d(CYCLE_DELIVER_X, CYCLE_DELIVER_Y, Math.PI - 0.05);
//        }
//        task.add(
//                new ParallelTask(
//                        new ClawTask(robot, ClawTask.ClawPosition.CLOSE),
//                        new SeriesTask(
//                                new SleepTask(300),
//                                new VerticalSlidesTask(robot, DELIVERY_CHAMBER_HEIGHT, 1.0, 2000)
//                        ),
//                        new SeriesTask(
//                                new SleepTask(700),
//                                new DeliveryArmTask(robot, DeliveryArmTask.Position.CHAMBER_DELIVER)
//                        ),
//                        new SeriesTask(
//                                new SleepTask(200),
//                                new RuntimeDrivingTask(
//                                        robot,
//                                        builder -> builder.strafeToLinearHeading(deliveryPose.position, deliveryPose.heading, new TranslationalVelConstraint(60)).build()
//                                )
//                        )
//                )
//        );
//        presetTask = task;
//    }
//}
