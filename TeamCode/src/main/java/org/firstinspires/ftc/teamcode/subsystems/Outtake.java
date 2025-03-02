package org.firstinspires.ftc.teamcode.subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

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
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

@Config
public class Outtake implements Subsystem {
    public static final Outtake INSTANCE = new Outtake();

    public static DcMotor slideLeft, slideRight;
    public static Servo elbowLeft, elbowRight, wrist, claw;

    public static TouchSensor outLimit;
    public static final int SLIDE_TOLERANCE = 50;

    private Outtake() {}

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

        slideLeft = hardwareMap.get(DcMotorEx.class, "OSL");
        slideRight = hardwareMap.get(DcMotorEx.class, "OSR");
        outLimit = hardwareMap.get(TouchSensor.class, "outLimit");

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideLeft.setPower(1);
        slideRight.setPower(1);
        slideLeft.setTargetPosition(0);
        slideRight.setTargetPosition(0);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        servo = hardwareMap.get(Servo.class, "outServo");
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

    public static void setSpeed(double speed) {
        slideLeft.setPower(speed);
        slideRight.setPower(speed);
    }
    public static Lambda outtakeSample() {
        return Lambda.from(new Sequential(
                        Claw.closeClaw(),
                        slideOut(),
                        Claw.elbowOut(),
                        Claw.openClaw(),
                        Claw.elbowIn()

                ))
                .addRequirements(Claw.INSTANCE)
                ;
    }
    public static Lambda outtakeSpecimen() {
        return Lambda.from(new Sequential(
                new Parallel(
                    slideTo(-2000),
                    new Sequential(
                        new Wait(0.32),
                        new Parallel(
                            Claw.openClaw(),
                            Claw.elbowIn()
                        )
                    )
                )
        ))
                .addRequirements(Claw.INSTANCE)
                ;
    }
    public static Lambda slowOuttakeSpecimen() {
        return Lambda.from(new Sequential(
                        new Sequential(
                                slideTo(-1900),
                                new Sequential(
                                        new Wait(0.4),
                                        new Parallel(
                                                Claw.openClaw(),
                                                Claw.elbowIn()
                                        )
                                )
                        )
                ))
                .addRequirements(Claw.INSTANCE)
                ;
    }


    public static Lambda slideTo(int encoderPos) {
        return new Lambda("slide-to")
                .setInit(() -> {
                    slideLeft.setTargetPosition(encoderPos);
                    slideRight.setTargetPosition(encoderPos);
                    slideLeft.setPower(1);
                    slideRight.setPower(1);
                    setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .setExecute(()->{
                    if(outLimit.isPressed() && slideLeft.getCurrentPosition() < slideLeft.getTargetPosition()){
                        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        slideLeft.setTargetPosition(0);
                        slideRight.setTargetPosition(0);
                        setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                })
                .setFinish(() -> {
                    return Math.abs(slideLeft.getTargetPosition() - slideLeft.getCurrentPosition()) < SLIDE_TOLERANCE || (outLimit.isPressed() && slideLeft.getTargetPosition() > slideLeft.getCurrentPosition());
                })
                .addRequirements(slideLeft, slideRight)
                ;

    }

    public static Lambda slideOut() {
        return slideTo(-4000);
    }

    public static Lambda slideIn() {
        return slideTo(-10);
    }

}