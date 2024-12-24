package org.firstinspires.ftc.teamcode.auto.defaultauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.V2Robot;
import org.firstinspires.ftc.teamcode.drive.V2RobotBase;

@Config

@Autonomous(name = "V2 Robot - CLIP Auto")
public class ClipAuto extends ClipAutoPath {

    public static double SUBMERSIBLE_DELIVER_DIST = -31.6, DELIVER_OFFSET = -0.4;
    public static double preloadSubmersibleYVal = 3.0, cycleSubmersibleYVal = -3.0;


    @Override
    protected V2RobotBase createRobot(HardwareMap hardwareMap) {
        return new V2Robot(hardwareMap);
    }

    @Override
    protected Pose2d getPreloadSubmersibleDeliverPos() {
        return new Pose2d(-32.50, preloadSubmersibleYVal, 0.0);
    }

    @Override
    protected Pose2d getFirstSampleIntakePos() {
        return new Pose2d(-27.65, 19.79, 2.04256);
    }

    @Override
    protected Pose2d getFirstSampleOuttakePos() {
        return new Pose2d(-20.65, 19.79, 0.62266);
    }

    @Override
    protected Pose2d getSecondSampleIntakePos() {
        return new Pose2d(-25.42, 29.29, 2.09439);
    }

    @Override
    protected Pose2d getSecondSampleOuttakePos() {
        return new Pose2d(-25.42, 29.29, Math.PI/4-0.1);
    }

    @Override
    protected Pose2d getThirdSampleIntakePos() {
        return new Pose2d(-21.69, 42.96, new Rotation2d(-0.7593, 0.6508).toDouble());
    }

    @Override
    protected Pose2d getThirdSampleOuttakePos() {
        return new Pose2d(-18.60, 38.46, 0.69813-0.1);
    }

    @Override
    protected Pose2d getSpecimenIntakePos(int cycleNumber) {
        return new Pose2d(0.5+0.05*cycleNumber, 30.56, -Math.PI+0.05);
    }

    @Override
    protected Pose2d getSubmersibleDeliverPos(int cycleNumber) {
        return new Pose2d(SUBMERSIBLE_DELIVER_DIST+cycleNumber*DELIVER_OFFSET, cycleSubmersibleYVal-cycleNumber*1.5, -0.05);
    }

    @Override
    protected Pose2d getParkPos() {
        return new Pose2d(-17.48, 16.55, 0.8561);
    }
}
