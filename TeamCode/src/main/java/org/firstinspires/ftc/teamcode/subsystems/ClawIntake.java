package org.firstinspires.ftc.teamcode.subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

@Config
public class ClawIntake implements Subsystem {
    public static final ClawIntake INSTANCE = new ClawIntake();

    public static DcMotorEx slideLeft, slideRight;
    public static CRServo sideSpinL, sideSpinR;
    public static Servo lElbow, rElbow;
    public static SampleDetector sampleDetector;

    // TODO: adjust to fit
    /// changes how long servo actions should wait until reporting they are complete
    public static Telemetry telemetry;
    public static final double SERVO_DELAY = 0.4;
    public static final int SLIDE_TOLERANCE = 50;

    private ClawIntake() {
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

        sampleDetector = new SampleDetector(hardwareMap);

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

                    slideLeft.setPower(1);
                    slideRight.setPower(1);
                    slideLeft.setTargetPosition(target);
                    slideRight.setTargetPosition(target);
//                    if(slideLeft.getMode() == DcMotor.RunMode.RUN_USING_ENCODER)
                        setMode(DcMotor.RunMode.RUN_TO_POSITION);

//                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .setExecute(()->{
//                    telemetry.addData("slideLeft Position", slideLeft.getCurrentPosition());
//                    slideLeft.getCurrent(CurrentUnit.AMPS)
                })
                .setFinish(() -> {
                    return Math.abs(target - slideLeft.getCurrentPosition()) < SLIDE_TOLERANCE;
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
                .addRequirements(lElbow, rElbow)
//                .setFinish(()->{
//                    return Math.abs(lElbow.getPosition() - pos) < 0.001;
//                })
//                new Wait(SERVO_DELAY)
        ))
                ;
    }

    public static Lambda elbowOut() {
        return elbowTo(0.09);
    }

    public static Lambda elbowIn() {
        return elbowTo(0.85);
    }


}