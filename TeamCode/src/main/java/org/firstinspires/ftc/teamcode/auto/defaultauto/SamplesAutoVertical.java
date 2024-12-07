package org.firstinspires.ftc.teamcode.auto.defaultauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.V2Robot;
import org.firstinspires.ftc.teamcode.drive.V2RobotBase;
@Config
@Autonomous(name = "V2 Robot - SAMPLE Auto Vertical")
public class SamplesAutoVertical extends SamplesAutoPath {

    public static double Sample1IntakeAlignPosX = 22.0738;
    public static double Sample1IntakeAlignPosY = 15.7592;
    public static double Sample2IntakeAlignPosX = 18.0738;
    public static double Sample2IntakeAlignPosY = 25.6237;
    public static double Sample3IntakeAlignPosX = 37.0852;
    public static double Sample3IntakeAlignPosY = 13.3009;

    public static int INTAKE_DELAY = 2000;
    @Override
    protected V2RobotBase createRobot(HardwareMap hardwareMap) {
        return new V2Robot(hardwareMap);
    }

    @Override
    protected Pose2d getPreloadBucketAlignPos() {
        return new Pose2d(8.2327, 23.6875, -Math.PI/4);
    }

    @Override
    protected Pose2d getPreloadBucketDeliverPos() {
        return new Pose2d(5.9913, 25.6989, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample1IntakeAlignPos() {
        return new Pose2d(Sample1IntakeAlignPosX, Sample1IntakeAlignPosY, 0);
    }

    @Override
    protected Pose2d getSample1IntakePos() {
        return new Pose2d(37.0738, 6.5973, Math.PI/2);
    }

    @Override
    protected Pose2d getSample1BucketAlignPos() {
        return new Pose2d(7.1342, 20.1532, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample1BucketDeliverPos() {
        return new Pose2d(5.1532, 23.2903, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample2IntakeAlignPos() {
        return new Pose2d(Sample2IntakeAlignPosX, Sample2IntakeAlignPosY, 0);
    }

    @Override
    protected Pose2d getSample2IntakePos() {
        return new Pose2d(35.9134, 13.2687, Math.PI/2);
    }

    @Override
    protected Pose2d getSample2BucketAlignPos() {
        return new Pose2d(6.1709, 19.5823, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample2BucketDeliverPos() {
        return new Pose2d(3.6991, 20.6653, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample3IntakeAlignPos() {
        return new Pose2d(Sample3IntakeAlignPosX, Sample3IntakeAlignPosY, Math.PI/2);
    }

    @Override
    protected Pose2d getSample3IntakePos() {
        return new Pose2d(35.0852, 17.4767, Math.PI/2);
    }

    @Override
    protected Pose2d getSample3BucketAlignPos() {
        return new Pose2d(6.4855, 17.4402, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample3BucketDeliverPos() {
        return new Pose2d(4.6098, 19.991, -Math.PI/4);
    }

    @Override
    protected Pose2d getParkAlignPos() {
        return getParkPos();
    }

    @Override
    protected Pose2d getParkPos() {
        return new Pose2d(51.906, -8.8614, Math.PI/2);
    }

    @Override
    protected int getIntakeDelay() {
        return INTAKE_DELAY;
    }
}
