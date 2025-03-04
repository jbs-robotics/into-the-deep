package org.firstinspires.ftc.teamcode.subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.auto.HT_PP;
import org.firstinspires.ftc.teamcode.driveClasses.ControlConstants;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Race;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

@Config
public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();

    //    public static DcMotorEx slideLeft, slideRight;
    public static CRServo sideSpinL, sideSpinR;
    public static Servo lElbow, rElbow, slideLeft, slideRight;

    // TODO: adjust to fit
    /// changes how long servo actions should wait until reporting they are complete
    public static Telemetry telemetry;
    public static final double SERVO_DELAY = 0.4;
    public static double elbowPosition = 0;
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
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {
    }

    public static Lambda sideSpinTo(double pos) {
        return new Lambda("side-spin-in")
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
                ;
    }

    public static Lambda sideSpinOut() {
        return new Lambda("side-spin-out")
                .addInit(() -> {
                    sideSpinL.setPower(1);
                    sideSpinR.setPower(-1);
                })
                .addRequirements(sideSpinL, sideSpinR)
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

    public static Lambda toggleElbow() {
        if (lElbow.getPosition() == ControlConstants.intakePivotIn) {
            return elbowOut();
        } else {
            return elbowIn();
        }
    }

    public static Lambda elbowTo(double pos) {
        elbowPosition = pos;
        return Lambda.from(new Sequential(
                new Lambda("intake-elbow-to")
                        .setInit(() -> {
                            lElbow.setPosition(pos);
                            rElbow.setPosition(1 - pos);
                        })
                        .addRequirements(lElbow, rElbow)
        ));
    }
    public static Lambda pushSlidesIn() {
        return new Lambda("push-intake-slides-in")
                .setExecute(() -> {
                    telemetry.addLine("pushing slides in");
                })
                .setFinish(() -> false)
                .setInterruptible(true);
    }
    public static Lambda randFunc() {
//        return new Lambda("rand")
//                .setExecute(() -> {
//                    elbowOut()
//                })
//                .setFinish(() -> false)
//                .setInterruptible(true)
//                ;
        return elbowOut();
    }

    public static Lambda elbowOut() {
        return elbowTo(0.09);
    }

    public static Lambda elbowIn() {
        return elbowTo(0.85);
    }


}