package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.V3Robot.V3Robot;
import org.firstinspires.ftc.teamcode.drive.V4Robot.V4Robot;
import org.firstinspires.ftc.teamcode.drive_rr.TestRobotDeadWheels;
import org.firstinspires.ftc.teamcode.drive_rr.TestRobotPinpoint;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
public class RobotFactory {
    public enum Robot {
        V3_ROBOT,
        V4_ROBOT,
        TEST_ROBOT_DEAD_WHEELS,
        TEST_ROBOT_PINPOINT;
    }
    public static Robot ACTIVE_ROBOT = Robot.V3_ROBOT;

    public static Robot getActiveRobot() {
        return ACTIVE_ROBOT;
    }

    public static MecanumDrive createRobot(HardwareMap hardwareMap) {
        switch (ACTIVE_ROBOT) {
            case TEST_ROBOT_DEAD_WHEELS:
                return new TestRobotDeadWheels(hardwareMap);
            case TEST_ROBOT_PINPOINT:
                return new TestRobotPinpoint(hardwareMap);
            case V3_ROBOT:
                return new V3Robot(hardwareMap);
            case V4_ROBOT:
            default:
                return new V4Robot(hardwareMap);
        }
    }
}
