package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
public class ClawIntake implements Subsystem {
    public static final ClawIntake INSTANCE = new ClawIntake();

    public static Servo clawServo, clawPivot; // declare the claw servos
    public static DcMotorEx slideLeft, slideRight; // declare the claw servos

    // TODO: adjust to fit
    /// changes how long servo actions should wait until reporting they are complete
    public static final double SERVO_DELAY = 0.4;

    private ClawIntake() {}

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
        clawPivot = hardwareMap.get(Servo.class, "clawPivot");

        slideLeft = hardwareMap.get(DcMotorEx.class, "ISL");
        slideLeft.setDirection(DcMotor.Direction.FORWARD);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideRight = hardwareMap.get(DcMotorEx.class, "ISR");
        slideRight.setDirection(DcMotor.Direction.REVERSE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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


}