package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
public class Claw implements Subsystem {
    public static final Claw INSTANCE = new Claw();

    public static Servo clawServo, lElbow, rElbow, wrist; // declare the claw servos

    // TODO: adjust to fit
    /// changes how long servo actions should wait until reporting they are complete
    public static final double SERVO_DELAY = 0.4;

    private Claw() {}

    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
    @Inherited
    public @interface Attach { }

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

        clawServo = hardwareMap.get(Servo.class, "claw");
        lElbow = hardwareMap.get(Servo.class, "outServoL");
        rElbow = hardwareMap.get(Servo.class, "outServoR");
        wrist = hardwareMap.get(Servo.class, "clawWrist");
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {}

    public static Lambda openClaw() {
        return Lambda.from(new Sequential(
                new Lambda("open-claw")
                .setInit(() -> clawServo.setPosition(0))
                .addRequirements(clawServo)
                ,
                new Wait(SERVO_DELAY)
        ));
    }
    public static Lambda closeClaw() {
        return Lambda.from(new Sequential(
            new Lambda("close-claw")
                .setInit(() -> clawServo.setPosition(0.5))
                .addRequirements(clawServo)
                ,
                new Wait(SERVO_DELAY)
        ));
    }

    public static Lambda elbowOut() {
        return Lambda.from(new Sequential(
        new Lambda("elbow-out")
                .setInit(() -> {
                    lElbow.setPosition(1);
                    rElbow.setPosition(0);
                })
                .addRequirements(rElbow, lElbow)
                ,
                new Wait(SERVO_DELAY)
        ));
    }
    public static Lambda elbowIn() {
        return Lambda.from(new Sequential(
            new Lambda("elbow-in")
                .setInit(() -> {
                    lElbow.setPosition(0);
                    rElbow.setPosition(1);
                })
                .addRequirements(rElbow, lElbow)
                ,
                new Wait(SERVO_DELAY)
        ));
    }

    public static Lambda elbowTo(double pos) {
        return Lambda.from(new Sequential(
            new Lambda("elbow-to")
                .setInit(() -> {
                    lElbow.setPosition(1-pos);
                    rElbow.setPosition(pos);
                })
                .addRequirements(rElbow, lElbow)
                ,
                new Wait(SERVO_DELAY)
        ));
    }
    public static Lambda wristIn(){
        return Lambda.from(new Sequential(
                new Lambda("wrist-to")
                        .setInit(()->{
                            wrist.setPosition(0);
                        })
                        .addRequirements(wrist)
        ));
    }
    public static Lambda wristOut(){
        return Lambda.from(new Sequential(
                new Lambda("wrist-to")
                        .setInit(()->{
                            wrist.setPosition(1);
                        })
                        .addRequirements(wrist)
        ));
    }
    public static Lambda wristTo(double pos){
        return Lambda.from(new Sequential(
                new Lambda("wrist-to")
                        .setInit(()->{
                            wrist.setPosition(pos);
                        })
                        .addRequirements(wrist)
        ));
    }

}