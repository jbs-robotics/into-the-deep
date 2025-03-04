package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotor;
import dev.frozenmilk.util.cell.Cell;


public class TeleopChassis extends SDKSubsystem {
    public static final TeleopChassis INSTANCE = new TeleopChassis();
    private static final double driveSensitivity = 1;

    public TeleopChassis() {
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @Inherited
    public @interface Attach{}
    //Dependencies for Mercurial
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


    //Hardware initialization
    private final Cell<CachingDcMotor> leftFront = subsystemCell(() -> new CachingDcMotor(getHardwareMap().get(DcMotor.class, "leftFront")));
    private final Cell<CachingDcMotor> leftBack = subsystemCell(() -> new CachingDcMotor(getHardwareMap().get(DcMotor.class, "leftBack")));
    private final Cell<CachingDcMotor> rightFront = subsystemCell(() -> new CachingDcMotor(getHardwareMap().get(DcMotor.class, "rightFront")));
    private final Cell<CachingDcMotor> rightBack = subsystemCell(() -> new CachingDcMotor(getHardwareMap().get(DcMotor.class, "rightBack")));


    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        leftFront.get().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.get().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.get().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.get().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.get().setDirection(DcMotor.Direction.REVERSE);
        leftBack.get().setDirection(DcMotor.Direction.REVERSE);
    }

    public Command defaultDriveCommand() {
        BoundGamepad gamepad1 = Mercurial.gamepad1();
        return new Lambda("default-drive-command")
                .setInit(() -> {
                })
                .setInterruptible(true)
                .addRequirements(INSTANCE)
                .setExecute(() -> {
                    double drivePower = gamepad1.leftStickY().state();
                    double turnPower = gamepad1.rightStickX().state();
                    double strafePower = gamepad1.leftStickX().state();

//                  TODO: Change if needed

                    double lfPower = Range.clip(drivePower + turnPower + strafePower, -driveSensitivity, driveSensitivity);
                    double rfPower = Range.clip(drivePower - turnPower - strafePower, -driveSensitivity, driveSensitivity);
                    double lbPower = Range.clip(drivePower + turnPower - strafePower, -driveSensitivity, driveSensitivity);
                    double rbPower = Range.clip(drivePower - turnPower + strafePower, -driveSensitivity, driveSensitivity);

                    leftFront.get().setPower(lfPower);
                    leftBack.get().setPower(lbPower);
                    rightFront.get().setPower(rfPower);
                    rightBack.get().setPower(rbPower);
//                    getTelemetry().addData("Drive power", drivePower);
//                    getTelemetry().addData("Turn Power", turnPower);
//                    getTelemetry().addData("Strafe Power", strafePower);

                })
                .setFinish(() -> false)
                ;
    }
}
