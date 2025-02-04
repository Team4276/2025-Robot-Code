package frc.team4276.util.path;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FlippingUtil;

public class PPUtil {
    public static PathPlannerTrajectory mirrorLengthWise(PathPlannerTrajectory trajectory){ //TODO: fix this abomination
        trajectory = trajectory.flip();
        FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kMirrored;
        trajectory = trajectory.flip();
        FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;

        return trajectory;
    }
    
}
