package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveClasses.ControlConstants;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
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
    public static double elbowPosition = 0, wristPosition = 0;

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
        lElbow.setDirection(Servo.Direction.REVERSE);
        wrist = hardwareMap.get(Servo.class, "clawWrist");
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {}
    public static Lambda clawTo(double pos){
        return Lambda.from(new Sequential(
                new Lambda("open-claw")
                        .setInit(() -> clawServo.setPosition(pos))
                        .addRequirements(clawServo)
        ));
    }
    public static Lambda toggleClaw(){
        if(clawServo.getPosition() == ControlConstants.clawClosed){
            return openClaw();
        }
        else{
            return closeClaw();
        }
    }
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

    public static Lambda toggleElbow() {
        if (elbowPosition == ControlConstants.outtakePivotOut) {
            return elbowIn();
        } else {
            return elbowOut();
        }
    }

    public static Lambda elbowOut() {
        return elbowTo(ControlConstants.outtakePivotOut);
//        return Lambda.from(new Sequential(
//        new Lambda("elbow-out")
//                .setInit(() -> {
//                    lElbow.setPosition(1);
//                    rElbow.setPosition(0);
//                })
//                .addRequirements(rElbow, lElbow)
//                ,
//                new Wait(SERVO_DELAY)
//        ));
    }
    public static Lambda elbowIn() {
        return elbowTo(ControlConstants.outtakePivotIn);
//        return Lambda.from(new Sequential(
//            new Lambda("elbow-in")
//                .setInit(() -> {
//                    lElbow.setPosition(0);
//                    rElbow.setPosition(1);
//                })
//                .addRequirements(rElbow, lElbow)
//                ,
//                new Wait(SERVO_DELAY)
//        ));
    }

    public static Lambda elbowTo(double pos) {
        elbowPosition = pos;
        return Lambda.from(new Sequential(
            new Lambda("elbow-to")
                .setInit(() -> {
                    lElbow.setPosition(pos);
                    rElbow.setPosition(pos);
                })
                .addRequirements(rElbow, lElbow)
                ,
                new Wait(SERVO_DELAY)
        ));
    }
    public static Lambda wristBack(){
        return wristTo(ControlConstants.outtakeWristBack);

//        return Lambda.from(new Sequential(
//                new Lambda("wrist-to")
//                        .setInit(()->{
//                            wrist.setPosition(0);
//                        })
//                        .addRequirements(wrist)
//        ));
    }
    public static Lambda wristForward(){
        return wristTo(ControlConstants.outtakeWristForward);
//        return Lambda.from(new Sequential(
//                new Lambda("wrist-to")
//                        .setInit(()->{
//                            wrist.setPosition(1);
//                        })
//                        .addRequirements(wrist)
//        ));
    }
    public static Lambda incrementWrist(double amt){
        return wristTo(wristPosition + amt);
    }
    public static Lambda wristTo(double pos){
        wristPosition = pos;
                return new Lambda("wrist-to")
                        .setInit(()->{
                            wrist.setPosition(pos);
                        })
                        .setExecute(() -> {
                        })
                        .setFinish(() ->
                             Math.abs(pos - wrist.getPosition()) < 0.1
                        )
                        .addRequirements(wrist);
    }

    public static Lambda gamepadWristMove(double modifier) {
        return new Lambda("gamepad-wrist-move")
                .setExecute(() -> {
                    double target = Range.clip(wristPosition + ControlConstants.outtakeWristSensitivity * modifier, ControlConstants.outtakeWristBack, ControlConstants.outtakeWristForward);
                    wristPosition = target;
                    FeatureRegistrar.getActiveOpMode().telemetry.addData("outtake target", target);
                    FeatureRegistrar.getActiveOpMode().telemetry.update();

                    wrist.setPosition(target);
                })
                .addRequirements(wrist)
                .setInterruptible(true)
                ;
    }

}