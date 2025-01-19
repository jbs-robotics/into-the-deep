package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


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
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class Chassis implements Subsystem {
    public static final Chassis INSTANCE = new Chassis();
    public static Follower follower;

    public static DcMotorEx fl;
    public static DcMotorEx fr;
    public static DcMotorEx bl;
    public static DcMotorEx br;
    public static Telemetry telemetry;

    public Chassis() {}

    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
    @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        telemetry = opMode.getOpMode().telemetry;
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(FeatureRegistrar.getActiveOpMode().hardwareMap);



        // TODO: add options for non-spec starting positions
        follower.setStartingPose(new Pose(81 , 9, Math.toRadians(-90)));


        HardwareMap hMap = opMode.getOpMode().hardwareMap;
        fl = hMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        bl = hMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        fr = hMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);
        br = hMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
    }

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {}

    @Override
    public void preUserStartHook(@NonNull Wrapper opMode) {}

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {}

    public static Lambda followPath(Path path) {
        return new Lambda("follow-path")
                .addRequirements(INSTANCE)
                .setInterruptible(true)
                .setInit(() -> follower.followPath(path, true))
                .setExecute(() -> {
                    follower.update();
                })
                .setFinish(() -> !follower.isBusy())
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                });
    }

    public static Lambda followPathChain(PathChain chain) {
        return new Lambda("follow-path-chain")
                .addRequirements(INSTANCE)
                .setInterruptible(true)
                .setInit(() -> follower.followPath(chain, true))
                .setExecute(() -> {
                    follower.update();
                })
                .setFinish(() -> !follower.isBusy())
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                });
    }
}