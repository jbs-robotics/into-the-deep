package org.firstinspires.ftc.teamcode.subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.auto.HT_PP;
import org.firstinspires.ftc.teamcode.driveClasses.ControlConstants;

import java.awt.font.NumericShaper;
import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.bindings.BoundDoubleSupplier;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Race;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.stateful.StatefulLambda;
import dev.frozenmilk.mercurial.commands.util.IfElse;
import dev.frozenmilk.mercurial.commands.util.Wait;
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.util.cell.RefCell;
import kotlin.annotation.MustBeDocumented;

@Config
public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();

    //    public static DcMotorEx slideLeft, slideRight;
    public static CRServo sideSpinL, sideSpinR;
    public static Servo lElbow, rElbow, slideLeft, slideRight, wiper;

    // TODO: adjust to fit
    /// changes how long servo actions should wait until reporting they are complete
    public static Telemetry telemetry;
    public static final double SERVO_DELAY = 0.4;
    public static RefCell<Double> elbowPosition = new RefCell<Double>(0.0);
    public static RefCell<Double> wiperPosition = new RefCell<Double>(ControlConstants.wiperIn);

    public static double slidePosition = ControlConstants.intakeSlideIn;
    public static final double SLIDE_TOLERANCE = 0.005;

    private Intake() {
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach {
    }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        Telemetry telemetry = opMode.getOpMode().telemetry;

        HardwareMap hardwareMap = opMode.getOpMode().hardwareMap;

        slideLeft = hardwareMap.get(Servo.class, "ISL");
        slideRight = hardwareMap.get(Servo.class, "ISR");

        slideLeft.setDirection(Servo.Direction.FORWARD);
        slideRight.setDirection(Servo.Direction.REVERSE);

        lElbow = hardwareMap.get(Servo.class, "inL");
        rElbow = hardwareMap.get(Servo.class, "inR");
        lElbow.setDirection(Servo.Direction.REVERSE);
        sideSpinL = hardwareMap.get(CRServo.class, "sideSpinL");
        sideSpinR = hardwareMap.get(CRServo.class, "sideSpinR");
        wiper = hardwareMap.get(Servo.class, "wiper");
        wiper.setPosition(wiperPosition.get());
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {
    }

    public static Lambda sideSpinTo(double pos) {
        return new Lambda("side-spin-to")
                .addInit(() -> {
                    sideSpinL.setPower(-pos);
                    sideSpinR.setPower(pos);
                })
                .addRequirements(sideSpinL, sideSpinR)
                ;
    }

    public static Lambda sideSpinIn() {
        return new Lambda("side-spin-in")
                .addInit(() -> {
                    sideSpinL.setPower(-1);
                    sideSpinR.setPower(1);
                })
                .addRequirements(sideSpinL, sideSpinR)
                .setEnd(interrupted -> {
                    if (interrupted) {
                        sideSpinR.setPower(0);
                        sideSpinL.setPower(0);
                    }
                })
                ;
    }

    public static Lambda sideSpinOut() {
        return new Lambda("side-spin-out")
                .addInit(() -> {
                    sideSpinL.setPower(1);
                    sideSpinR.setPower(-1);
                })
                .addRequirements(sideSpinL, sideSpinR)
                .setEnd(interrupted -> {
                    if (interrupted) {
                        sideSpinR.setPower(0);
                        sideSpinL.setPower(0);
                    }
                })
                ;
    }

    public static Lambda sideSpinOff() {
        return new Lambda("side-spin-off")
                .setInit(() -> {
                    sideSpinL.setPower(0);
                    sideSpinR.setPower(0);
                })
                .addRequirements(sideSpinL, sideSpinR)
                ;
    }

    public static Lambda slideTo(double target) {
        return new Lambda("intake-slide-to")
                .setInit(() -> {
                    slideLeft.setPosition(target);
                    slideRight.setPosition(target);
                })
                .setExecute(() -> {
                })
                .setFinish(() -> {
                    return Math.abs(target - slideLeft.getPosition()) < SLIDE_TOLERANCE;
                })
                .addRequirements(slideLeft, slideRight)
                ;
    }

    public static Lambda slideIn() {
        return Lambda.from(slideTo(ControlConstants.intakeSlideIn));
    }

    public static Lambda slideOut() {
        return Lambda.from(slideTo(ControlConstants.intakeSlideOut));
    }


    public static Command toggleElbow() {
        return new StatefulLambda<RefCell<Double>>("toggle-elbow", new RefCell<Double>(ControlConstants.intakePivotIn))
                .setExecute(refState -> {

                    double target;
                    if (refState.get() == ControlConstants.intakePivotIn) {
                        target = ControlConstants.intakePivotOut;
                    } else {
                        target = ControlConstants.intakePivotIn;
                    }
                    refState.accept(target);
                    elbowPosition.accept(target);
                    lElbow.setPosition(target);
                    rElbow.setPosition(1-target);
//                    FeatureRegistrar.getActiveOpMode().telemetry.addData("elbowPosition", elbowPosition.get());
//                    FeatureRegistrar.getActiveOpMode().telemetry.addData("refstate", refState.get());
//                    FeatureRegistrar.getActiveOpMode().telemetry.addData("Relbo position", rElbow.getPosition());
//                    FeatureRegistrar.getActiveOpMode().telemetry.update();
//                    elbowPosition = new RefCell<Double>(target);
                })
                .setRequirements(lElbow, rElbow)
                .setInit(() -> {
                    elbowPosition.accept(ControlConstants.intakePivotIn);
//                    elbowPosition = ControlConstants.intakePivotIn;
                    lElbow.setPosition(ControlConstants.intakePivotIn);
                    rElbow.setPosition(1-ControlConstants.intakePivotIn);
                })

                ;
    }

    public static Lambda elbowTo(double pos) {
        elbowPosition.accept(pos);
//        elbowPosition = pos;

                return new Lambda("intake-elbow-to")
                        .setExecute(() -> {
//                            lElbow.setPosition(pos);
                            rElbow.setPosition(1 - pos);
                        })
                        .addRequirements(lElbow, rElbow)
                        .setFinish(() -> true)
                        .setInterruptible(true)
                ;
    }
    public static Lambda pushSlidesIn() {
        return new Lambda("push-intake-slides-in")
                .setExecute(() -> {
                    double target = Range.clip(slidePosition + ControlConstants.intakeSlideSensitivity, ControlConstants.intakeSlideOut, ControlConstants.intakeSlideIn);
                    slidePosition = target;
                    slideLeft.setPosition(target);
                    slideRight.setPosition(target);
                })
                .setFinish(() -> false)
                .setInterruptible(true);
    }

    public static Lambda pushSlidesOut() {
        return new Lambda("push-intake-slides-out")
                .setExecute(() -> {
                    double target = Range.clip(slidePosition - ControlConstants.intakeSlideSensitivity, ControlConstants.intakeSlideOut, ControlConstants.intakeSlideIn);
                    slidePosition = target;
                    slideLeft.setPosition(target);
                    slideRight.setPosition(target);
                })
                .setFinish(() -> false)
                .setInterruptible(true);
    }
    public static Lambda pushSlidesOut(double sens) {
        return new Lambda("push-intake-slides-out")
                .setExecute(() -> {
                    double target = Range.clip(slidePosition - sens, ControlConstants.intakeSlideOut, ControlConstants.intakeSlideIn);
                    slidePosition = target;
                    slideLeft.setPosition(target);
                    slideRight.setPosition(target);
                })
                .setFinish(() -> slidePosition <= ControlConstants.intakeSlideOut || slidePosition >= ControlConstants.intakeSlideIn)
                .setInterruptible(true);
    }


    public static Lambda elbowOut() {
        return elbowTo(ControlConstants.intakePivotOut);
    }

    public static Lambda elbowIn() {
        return elbowTo(ControlConstants.intakePivotIn);
    }

    public static Lambda toggleWiper() {
        return new Lambda("toggle-wiper")
                .setExecute(() -> {
                    double target;
                    if (wiperPosition.get() == ControlConstants.wiperIn) {
                        target = ControlConstants.wiperOut;
                    } else {
                        target = ControlConstants.wiperIn;
                    }
                    wiperPosition.accept(target);
                    wiper.setPosition(target);
                })
                .setRequirements(wiper)
                ;
    }
    public static Lambda wiperToPos(BoundDoubleSupplier supplier) {
        return new Lambda("wiper-to-pos")
                .setExecute(() -> {
                    wiperPosition.accept(supplier.state());
                    wiper.setPosition(wiperPosition.get());
                })
                .setFinish(() -> false)
                .setRequirements(wiper)
                ;
    }

}