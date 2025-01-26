package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.SampleColor;

@TeleOp(name = "Red Exp TeleOp")
public class RedExpTeleOp extends ExpTeleOp {
    @Override
    protected SampleColor getTargetColor() {
        return SampleColor.COLOR_NOT_BLUE;
    }
}
