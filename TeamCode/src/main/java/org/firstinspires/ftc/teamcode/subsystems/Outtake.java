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
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

@Config
public class Outtake implements Subsystem {
    public static final Outtake INSTANCE = new Outtake();

    // TODO: fine-tune targetTolerance
    /// Used to determine when slides are at desired position. If slide position is within targetTolerance of target position, actions will be considered complete.
    private int targetTolerance = 100;

    public static DcMotor slideLeft, slideRight;

    public static TouchSensor outLimit;

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


        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
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
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                slideTo(-1900),
                Claw.openClaw(),
                Claw.elbowIn()
        ))
                .addRequirements(Claw.INSTANCE)
                ;
    }


    public static Lambda slideTo(int encoderPos) {
        return new Lambda("slide-to")
                .setInit(() -> {
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideLeft.setTargetPosition(encoderPos);
                    slideRight.setTargetPosition(encoderPos);
                    slideLeft.setPower(1);
                    slideRight.setPower(1);
                })
                .setFinish(() -> {
                    return !slideLeft.isBusy() && !slideLeft.isBusy();
                })
                .addRequirements(slideLeft, slideRight)
                ;

    }

    public static Lambda slideOut() {
        return new Lambda("slide-out")
                .setInit(() -> {
                    slideLeft.setTargetPosition(-4000);
                    slideRight.setTargetPosition(-4000);
                    if(slideLeft.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                        slideLeft.setPower(1);
                        slideRight.setPower(1);
                        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                })
                .setFinish(() -> {
                    // TODO: Ken let me know if this is me interpreting you correctly in auto/Outtake.java:163
                    return (!slideLeft.isBusy() && !slideLeft.isBusy()) || slideLeft.getCurrentPosition() > -4000 || slideRight.getCurrentPosition() > -4000;
                })
                .addRequirements(slideLeft, slideRight)
                ;
    }

    public static Lambda slideIn() {
        return new Lambda("slide-in")
                .setInit(() -> {
                    slideLeft.setTargetPosition(0);
                    slideRight.setTargetPosition(0);
                    if (slideLeft.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                        slideLeft.setPower(1);
                        slideRight.setPower(1);
                        setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                })
                .setFinish(() -> {
                    return !slideLeft.isBusy() && !slideLeft.isBusy();
                })
                .addRequirements(slideLeft, slideRight)
                ;
    }

}