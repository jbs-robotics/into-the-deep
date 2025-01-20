package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.stateful.StatefulLambda;
import dev.frozenmilk.util.cell.RefCell;
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
import java.util.function.Supplier;

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


    }

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {}

    @Override
    public void preUserStartHook(@NonNull Wrapper opMode) {}

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {}

    public static Lambda followPath(Path path, boolean holdEnd) {
        return new Lambda("follow-path")
                .addRequirements(INSTANCE)
                .setInterruptible(true)
                .setInit(() -> follower.followPath(path, holdEnd))
                .setExecute(() -> {
                    follower.update();
                    telemetry.addData("Pose: ", follower.getPose());
                    telemetry.update();
                })
                .setFinish(() -> !follower.isBusy())
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                });
    }

    public static Lambda followPath(PathChain chain, boolean holdEnd) {
        return new Lambda("follow-path-chain")
                .addRequirements(INSTANCE)
                .setInterruptible(true)
                .setInit(() -> follower.followPath(chain, holdEnd))
                .setExecute(() -> {
                    follower.update();
                    telemetry.addData("Pose: ", follower.getPose());
                    telemetry.update();
                })
                .setFinish(() -> {
                    return !follower.isBusy();
//                    return chain.getPath(chain.size()-1).getLastControlPoint().getX() - follower.getPose().getX() < 1 && chain.getPath(chain.size()-1).getLastControlPoint().getY() - follower.getPose().getY() < 1;
                })
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                });
    }
    public static Lambda followPath(PathChain chain, int tolerance) {
        return new Lambda("follow-path-chain")
                .addRequirements(INSTANCE)
                .setInterruptible(true)
                .setInit(() -> follower.followPath(chain, true))
                .setExecute(() -> {
                    follower.update();
                    telemetry.addData("Pose: ", follower.getPose());
                    telemetry.update();
                })
                .setFinish(() -> {
//                    return
                    return chain.getPath(chain.size()-1).getLastControlPoint().getX() - follower.getPose().getX() < tolerance && chain.getPath(chain.size()-1).getLastControlPoint().getY() - follower.getPose().getY() < tolerance;
                })
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                });
    }

//    public static Lambda followPath(PathChain chain, @NonNull Supplier<Boolean> pathEndAt, boolean holdEnd, Supplier<Boolean> actionAt, Lambda action) {
//        return new StatefulLambda<RefCell<Boolean>>("follow-path-chain", new RefCell<Boolean>(false))
//                .addRequirements(INSTANCE)
//                .setInterruptible(true)
//                .setInit(() -> follower.followPath(chain, holdEnd))
//                .setExecute((stateRef) -> {
//                    follower.update();
//                    if (actionAt.get() && !stateRef.get()) {
//                        stateRef.accept(true);
//                        action.schedule();
//                    }
//                })
//                .setFinish(pathEndAt)
//                .setEnd((interrupted) -> {
//                    if (interrupted) follower.breakFollowing();
//                });
//    }

    public static Lambda holdPoint(Pose point) {
        return new Lambda("hold-point")
                .addRequirements(INSTANCE)
                .setInterruptible(true)
                .setInit(() -> follower.holdPoint(point))
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                })
                ;
    }

    public static Lambda holdPoint(BezierPoint point, double heading){
        return new Lambda("hold-point")
                .addRequirements(INSTANCE)
                .setInterruptible(true)
                .setInit(() -> follower.holdPoint(point, heading))
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                })
                ;
    }

    public static Lambda holdPoint(Point point, double heading) {
        return new Lambda("hold-point")
                .addRequirements(INSTANCE)
                .setInterruptible(true)
                .setInit(() -> follower.holdPoint(point, heading))
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                })
                ;
    }
}