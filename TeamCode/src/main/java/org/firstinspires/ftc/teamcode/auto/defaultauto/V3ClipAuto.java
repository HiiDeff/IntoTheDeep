package org.firstinspires.ftc.teamcode.auto.defaultauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.V2Robot;

@Config

@Autonomous(name = "V3 CLIP Auto")
public class V3ClipAuto extends ClipAutoPath {

    public static double SUBMERSIBLE_DELIVER_DIST = -31.5, DELIVER_OFFSET = -0.0;
    public static double preloadSubmersibleYVal = 3.0, cycleSubmersibleYVal = -1.0, HORIZONTAL_DELIVER_OFFSET = -1.9;


    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap); //new V2Robot(hardwareMap);
    } //TODO ask abt this

    @Override
    protected Pose2d getPreloadSubmersibleDeliverPos() {
        return new Pose2d(-32.50, preloadSubmersibleYVal, 0.0);
    }

    @Override
    protected Pose2d getFirstSampleIntakePos() {
        return new Pose2d(-20.962, 17.441, 2.2708);
    }

    @Override
    protected Pose2d getFirstSampleOuttakePos() {
        return new Pose2d(-20.962, 17.441, 0.8298);
    }

    @Override
    protected Pose2d getSecondSampleIntakePos() {
        return new Pose2d(-23.6228, 30.1771, 2.1692);
    }

    @Override
    protected Pose2d getSecondSampleOuttakePos() {
        return new Pose2d(-23.6228, 30.1771, 0.7202);
    }

    @Override
    protected Pose2d getThirdSampleIntakePos() {
        return new Pose2d(-22.6415, 37.7287, 2.0426);
    }

    @Override
    protected Pose2d getThirdSampleOuttakePos() {
        return new Pose2d(-20.4855, 35.1955, 0.6121);
    }

    @Override
    protected Pose2d getSpecimenIntakePos(int cycleNumber) {
        return new Pose2d(0.5+0.05*cycleNumber, 30.56, -Math.PI+0.05); //x = 0.5+0.05*cycleNumber for V2 Robot
    }

    @Override
    protected Pose2d getSubmersibleDeliverPos(int cycleNumber) {
        return new Pose2d(SUBMERSIBLE_DELIVER_DIST+cycleNumber*DELIVER_OFFSET, cycleSubmersibleYVal+cycleNumber*HORIZONTAL_DELIVER_OFFSET, -0.05);
    }

    @Override
    protected Pose2d getParkPos() {
        return new Pose2d(-17.48, 16.55, 0.8561);
    }
}
