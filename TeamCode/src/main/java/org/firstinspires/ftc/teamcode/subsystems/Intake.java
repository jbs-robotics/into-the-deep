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
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

@Config
public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();

    public static DcMotorEx slideLeft, slideRight;
    public static CRServo sideSpinL, sideSpinR;
    public static Servo lElbow, rElbow;

    // TODO: adjust to fit
    /// changes how long servo actions should wait until reporting they are complete
    public static final double SERVO_DELAY = 0.4;

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
        HardwareMap hardwareMap = opMode.getOpMode().hardwareMap;

        slideLeft = hardwareMap.get(DcMotorEx.class, "ISL");
        slideLeft.setDirection(DcMotor.Direction.FORWARD);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slideLeft.setPower(1);

        slideRight = hardwareMap.get(DcMotorEx.class, "ISR");
        slideRight.setDirection(DcMotor.Direction.REVERSE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slideLeft.setPower(1);
        lElbow = hardwareMap.get(Servo.class, "inL");
        rElbow = hardwareMap.get(Servo.class, "inR");

        sideSpinL = hardwareMap.get(CRServo.class, "sideSpinL");
        sideSpinR = hardwareMap.get(CRServo.class, "sideSpinR");
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {
    }

    public static void resetEncoders(){
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setTargetPosition(0);
        slideRight.setTargetPosition(0);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static void setMode(DcMotor.RunMode runMode){
        slideLeft.setMode(runMode);
        slideRight.setMode(runMode);
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

    public static Lambda slideTo(int target) {
        return new Lambda("intake-slide-to")
                .setInit(() -> {
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideLeft.setPower(1);
                    slideRight.setPower(1);
                    slideLeft.setTargetPosition(target);
                    slideRight.setTargetPosition(target);
                })
                .setFinish(() -> {
                    return !slideLeft.isBusy() && !slideRight.isBusy();
                })
                .addRequirements(slideLeft, slideRight)
                ;
    }

    public static Lambda slideIn() {
        return Lambda.from(slideTo(0));
    }

    public static Lambda slideOut() {
        return Lambda.from(slideTo(1000));
    }



    public static Lambda elbowTo(double pos) {
        return Lambda.from(new Sequential(
                new Lambda("intake-elbow-to")
                .setInit(() -> {
                    lElbow.setPosition(pos);
                    rElbow.setPosition(1-pos);
                })
                .addRequirements(lElbow, rElbow),
                new Wait(SERVO_DELAY)
        ))
                ;
    }

    public static Lambda elbowOut() {
        return elbowTo(0.1150);
    }

    public static Lambda elbowIn() {
        return elbowTo(0.9);
    }


}