package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
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
@Outtake.Attach
@Claw.Attach
public class MilkyTeleOp extends OpMode {


    @Override
    public void init() {
        TeleopChassis.INSTANCE.setDefaultCommand(TeleopChassis.INSTANCE.defaultDriveCommand());

        // NOTE: Intake
        // Intake Slides
        Mercurial.gamepad2().rightBumper().whileTrue(Intake.pushSlidesOut());
        Mercurial.gamepad2().leftBumper().whileTrue(Intake.pushSlidesIn());
        // Intake Pivot
        Mercurial.gamepad2().dpadLeft().onTrue(Intake.toggleElbow());
        // Intake Spinning
        Mercurial.gamepad2().dpadUp().onTrue(Intake.sideSpinOut()).onFalse(Intake.sideSpinOff());
        Mercurial.gamepad2().dpadDown().onTrue(Intake.sideSpinIn()).onFalse(Intake.sideSpinOff());
        // Windshield Wiper
        Mercurial.gamepad2().touchpad().onTrue(Intake.toggleWiper());
        Mercurial.gamepad2().touchpadFinger1X().conditionalBindState().greaterThan(0.05).bind().whileTrue(Intake.wiperToPos(Mercurial.gamepad2().touchpadFinger1X()));


        // NOTE: Outtake
        // Outtake Elbow
        Mercurial.gamepad2().cross().onTrue(Claw.toggleElbow());
        // Outtake Wrist
        Mercurial.gamepad2().rightStickY().conditionalBindState().greaterThan(0.05).bind().whileTrue(Claw.gamepadWristMove(Mercurial.gamepad2().rightStickY()));
        Mercurial.gamepad2().rightStickY().conditionalBindState().lessThan(-0.05).bind().whileTrue(Claw.gamepadWristMove(Mercurial.gamepad2().rightStickY()));
        // Outtake Claw
        Mercurial.gamepad2().circle().onTrue(Claw.toggleClaw());
        // Outtake Slides
        Mercurial.gamepad2().leftStickY().conditionalBindState().greaterThan(0.05).bind().whileTrue(Outtake.gamepadSlides(Mercurial.gamepad2().leftStickY()));
        Mercurial.gamepad2().leftStickY().conditionalBindState().lessThan(-0.05).bind().whileTrue(Outtake.gamepadSlides(Mercurial.gamepad2().leftStickY()));


    }

    @Override
    public void loop() {
//        telemetry.addData("Intake Elbow", Intake.elbowPosition);
//        telemetry.addData("IOuttake Wrist", Claw.elbowPosition);
//        telemetry.update();
    }
}
