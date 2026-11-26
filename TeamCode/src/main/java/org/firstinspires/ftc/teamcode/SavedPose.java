package org.firstinspires.ftc.teamcode;

public final class SavedPose {
    private SavedPose() {}

    public static final class Pose {
        public final double x;
        public final double y;
        public final double heading;

        public Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    private static volatile Pose pose;

    public static void save(double x, double y, double heading) {
        pose = new Pose(x, y, heading);
    }

    public static Pose consume() {
        Pose saved = pose;
        pose = null;
        return saved;
    }

    public static void clear() {
        pose = null;
    }
}