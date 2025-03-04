package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.TeleopChassis;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Sequential;

@TeleOp(name = "Milky TeleOp", group = "Linear OpMode")
@Mercurial.Attach
@TeleopChassis.Attach
@Intake.Attach
public class MilkyTeleOp extends OpMode {


    @Override
    public void init() {
        TeleopChassis.INSTANCE.setDefaultCommand(TeleopChassis.INSTANCE.defaultDriveCommand());


        Mercurial.gamepad2().rightBumper().whileTrue(Intake.pushSlidesOut());
        Mercurial.gamepad2().leftBumper().whileTrue(Intake.pushSlidesIn());
    }

    @Override
    public void loop() {

    }
}
