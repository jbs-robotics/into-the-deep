package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveClasses.ControlConstants;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.TeleopChassis;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

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
        Mercurial.gamepad2().triangle().onTrue(Intake.toggleElbow());
        // Intake Spinning
        Mercurial.gamepad2().square().onTrue(Intake.sideSpinOut()).onFalse(Intake.sideSpinOff());
        Mercurial.gamepad2().cross().onTrue(Intake.sideSpinIn()).onFalse(Intake.sideSpinOff());
        // Windshield Wiper
        Mercurial.gamepad2().touchpad().onTrue(Intake.toggleWiper());


        // NOTE: Outtake
        // Outtake Elbow
        Mercurial.gamepad2().dpadDown().onTrue(Claw.toggleElbow());
        // Outtake Wrist
        Mercurial.gamepad2().leftStickY().conditionalBindState().greaterThan(0.05).bind().whileTrue(Claw.gamepadWristMove(Mercurial.gamepad2().leftStickY()));
        Mercurial.gamepad2().leftStickY().conditionalBindState().lessThan(-0.05).bind().whileTrue(Claw.gamepadWristMove(Mercurial.gamepad2().leftStickY()));
        // Outtake Claw
        Mercurial.gamepad2().dpadRight().onTrue(Claw.toggleClaw());
        // Outtake Slides
        Mercurial.gamepad2().rightStickY().conditionalBindState().greaterThan(0.05).bind().whileTrue(Outtake.gamepadSlides(Mercurial.gamepad2().rightStickY()));
        Mercurial.gamepad2().rightStickY().conditionalBindState().lessThan(-0.05).bind().whileTrue(Outtake.gamepadSlides(Mercurial.gamepad2().rightStickY()));

        // NOTE: Routines
        // Get ready to score high chamber
        Mercurial.gamepad2().rightTrigger().conditionalBindState().greaterThan(0.05).bind().onTrue(readyScoreHighChamber());
        // Get ready to score high basket
        Mercurial.gamepad2().leftTrigger().conditionalBindState().greaterThan(0.05).bind().onTrue(readyScoreHighBasket());
        // Transfer
        Mercurial.gamepad2().dpadLeft().onTrue(transferSample());
        // Get ready to grab spec
        Mercurial.gamepad2().circle().onTrue(getReadyToGrabSpec());

    }

    @Override
    public void loop() {
//        telemetry.addData("Intake Elbow", Intake.elbowPosition);
//        telemetry.addData("IOuttake Wrist", Claw.elbowPosition);
//        telemetry.update();
    }


    public Command readyScoreHighChamber() {
        return new Parallel(
                Claw.closeClaw(),
                Outtake.slideTo(ControlConstants.highChamberSlidePos),
                Claw.elbowOut(),
                Claw.wristBack()
        );
    }

    public Command readyScoreHighBasket() {
        return new Parallel(
                Claw.closeClaw(),
                Outtake.slideTo(ControlConstants.highBasketSlidePos),
                Claw.elbowOut(),
                Claw.wristBack()
        );
    }

    public Command getReadyToGrabSpec() {
        return new Parallel(
                Claw.openClaw(),
                Intake.slideIn(),
                Intake.elbowIn(),
                Outtake.slideTo(ControlConstants.minOuttakeSlidePos),
                Claw.elbowTo(ControlConstants.outtakePivotIn),
                Claw.wristTo(ControlConstants.pickupOuttakeWrist)
        );
    }

    public Command transferSample() {
        return new Sequential(
                new Parallel(
                        Intake.elbowTo(ControlConstants.transferIntakePivotPos),
                        Intake.slideTo(ControlConstants.transferIntakeSlidePos),
                        Claw.elbowTo(ControlConstants.transferOuttakePivotPos),
                        Claw.wristTo(ControlConstants.transferOuttakeWristPos),
                        Outtake.slideTo(ControlConstants.transferOuttakeSlidePos-600).setFinish(() ->  true),
                        Claw.openClaw()
                ),
                new Wait(0.4),
                Outtake.slideTo(ControlConstants.transferOuttakeSlidePos),
                Claw.closeClaw(),
                new Wait(0.2),
                Outtake.incrementSlides(-400)
        );
    }
}
