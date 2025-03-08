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
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.bindings.BoundDoubleSupplier;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.util.cell.RefCell;
import kotlin.annotation.MustBeDocumented;

@Config
public class Claw implements Subsystem {
    public static final Claw INSTANCE = new Claw();
    public static RefCell<Double> elbowPosition;

    public static Servo clawServo, lElbow, rElbow, wrist; // declare the claw servos
    public static RefCell<Double> wristPosition;
    public static RefCell<Double> clawPosition;

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
        if (hardwareMap == null) {
            hardwareMap = FeatureRegistrar.getActiveOpMode().hardwareMap;
            FeatureRegistrar.getActiveOpMode().telemetry.addLine("HardwareMap null for Claw; getting from feature registrar");
            FeatureRegistrar.getActiveOpMode().telemetry.update();
        }

        clawServo = hardwareMap.get(Servo.class, "claw");
        clawPosition = new RefCell<Double>(ControlConstants.clawClosed);
        clawServo.setPosition(clawPosition.get());

        lElbow = hardwareMap.get(Servo.class, "outServoL");
        rElbow = hardwareMap.get(Servo.class, "outServoR");
//        lElbow.setDirection(Servo.Direction.REVERSE);
        elbowPosition = new RefCell<Double>(ControlConstants.outtakePivotIn);
        lElbow.setPosition(elbowPosition.get());
        rElbow.setPosition(elbowPosition.get());

        wrist = hardwareMap.get(Servo.class, "clawWrist");
        wristPosition = new RefCell<Double>(ControlConstants.pickupOuttakeWrist);
        wrist.setPosition(wristPosition.get());
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
        return new Lambda("toggle-claw")
                .setExecute(() -> {
                    double target;
                    if (clawPosition.get() == ControlConstants.clawOpen) {
                        target = ControlConstants.clawClosed;
                    } else {
                        target = ControlConstants.clawOpen;
                    }
                    clawPosition.accept(target);
                    clawServo.setPosition(target);
                })
                .setRequirements(clawServo)
                ;
    }
    public static Lambda openClaw() {
        return new Lambda("open-claw")
                .setInit(() -> {
                    clawServo.setPosition(ControlConstants.clawOpen);
                    clawPosition.accept(ControlConstants.clawOpen);
                })
                .addRequirements(clawServo)
                ;
    }
    public static Lambda closeClaw() {
        return new Lambda("close-claw")
                .setInit(() -> {
                    clawServo.setPosition(ControlConstants.clawClosed);
                    clawPosition.accept(ControlConstants.clawClosed);
                })
                .addRequirements(clawServo)
                ;
    }

    public static Lambda toggleElbow() {
        return new Lambda("toggle-outtake-elbow")
                .setRequirements(rElbow, lElbow)
                .setInit(() -> {
                    double target;
                    if (elbowPosition.get() == ControlConstants.outtakePivotOut) {
                        target = ControlConstants.outtakePivotIn;
                    } else {
                        target = ControlConstants.outtakePivotOut;
                    }
                    elbowPosition.accept(target);
                    lElbow.setPosition(target);
                    rElbow.setPosition(target);
                })
                ;
    }

    public static Lambda elbowOut() {
        return elbowTo(ControlConstants.outtakePivotOut);
    }
    public static Lambda elbowIn() {
        return elbowTo(ControlConstants.outtakePivotIn);
    }

    public static Lambda elbowTo(double pos) {
        elbowPosition.accept(pos);
        return new Lambda("elbow-to")
                .setInit(() -> {
                    lElbow.setPosition(pos);
                    rElbow.setPosition(pos);
                })
                .addRequirements(rElbow, lElbow)
//                .setFinish(() -> Math.abs(lElbow.getPosition()) <= 0.1)
                .setFinish(() -> true) //TODO: FIX THIS WHEN THE AXONS GET PUT IN
                ;
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
        return wristTo(wristPosition.get() + amt);
    }
    public static Lambda wristTo(double pos){
        wristPosition.accept(pos);
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

    public static Lambda gamepadWristMove(BoundDoubleSupplier modifier) {
        return new Lambda("gamepad-wrist-move")
                .setExecute(() -> {
                    Telemetry tel = FeatureRegistrar.getActiveOpMode().telemetry;
                    tel.addData("Wiper target pos", modifier.state());
                    tel.update();
                    double target = Range.clip(wristPosition.get() + ControlConstants.outtakeWristSensitivity * modifier.state(), ControlConstants.outtakeWristBack, ControlConstants.outtakeWristForward);
                    wristPosition.accept(target);
                    wrist.setPosition(wristPosition.get());
                })
                .setFinish(() -> Math.abs(modifier.state()) <= 0.05)
                .addRequirements(wrist)
                .setInterruptible(true)
                ;
    }

}