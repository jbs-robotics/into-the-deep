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
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

@Config
public class Claw implements Subsystem {
    public static final Claw INSTANCE = new Claw();

    public static Servo clawServo, lElbow, rElbow; // declare the claw servos

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
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {}

    public static Lambda openClaw() {
        return new Lambda("open-claw")
                .setInit(() -> clawServo.setPosition(0))
                .addRequirements(clawServo)
                ;
    }
    public static Lambda closeClaw() {
        return new Lambda("close-claw")
                .setInit(() -> clawServo.setPosition(0.5))
                .addRequirements(clawServo)
                ;
    }

    public static Lambda elbowOut() {
        return new Lambda("elbow-out")
                .setInit(() -> {
                    lElbow.setPosition(1);
                    rElbow.setPosition(0);
                })
                .addRequirements(rElbow, lElbow)
                ;
    }
    public static Lambda elbowIn() {
        return new Lambda("elbow-in")
                .setInit(() -> {
                    lElbow.setPosition(0);
                    rElbow.setPosition(1);
                })
                .addRequirements(rElbow, lElbow)
                ;
    }

    public static Lambda elbowTo(double pos) {
        return new Lambda("elbow-to")
                .setInit(() -> {
                    lElbow.setPosition(1-pos);
                    rElbow.setPosition(pos);
                })
                .addRequirements(rElbow, lElbow)
                ;
    }
}