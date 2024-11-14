package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive_rr.TestRobotDeadWheels;
import org.firstinspires.ftc.teamcode.drive_rr.TestRobotPinpoint;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
public class RobotFactory {
    public enum Robot {
        LM1_ROBOT,
        V2_ROBOT,
        TEST_ROBOT_DEAD_WHEELS,
        TEST_ROBOT_PINPOINT;
    }
    public static Robot ACTIVE_ROBOT = Robot.V2_ROBOT;

    public static MecanumDrive createRobot(HardwareMap hardwareMap) {
        switch (ACTIVE_ROBOT) {
            case TEST_ROBOT_DEAD_WHEELS:
                return new TestRobotDeadWheels(hardwareMap);
            case TEST_ROBOT_PINPOINT:
                return new TestRobotPinpoint(hardwareMap);
            case LM1_ROBOT:
                return new LM1Robot(hardwareMap);
            case V2_ROBOT:
            default:
                return new V2Robot(hardwareMap);
        }
    }
}
