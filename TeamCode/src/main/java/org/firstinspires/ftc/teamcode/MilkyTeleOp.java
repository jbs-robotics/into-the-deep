package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TeleopChassis;

import dev.frozenmilk.mercurial.Mercurial;

@TeleOp(name = "Milky TeleOp", group = "Linear OpMode")
@Mercurial.Attach
@TeleopChassis.Attach
public class MilkyTeleOp extends OpMode {

    @Override
    public void init() {
        TeleopChassis.INSTANCE.setDefaultCommand(TeleopChassis.INSTANCE.defaultDriveCommand());
    }

    @Override
    public void loop() {

    }
}
