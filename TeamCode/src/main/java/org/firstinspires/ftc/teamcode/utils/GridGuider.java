package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

@Config
public class GridGuider {
    public static double G = 3;

    private double robotWidth;
    private double robotHeight;
    private ArrayList<Junction> junctions = new ArrayList<>();

    private class Junction {
        double x;
        double y;
        double radius;
        double distance;

        public Junction(double x, double y, double rad, double dist) {
            this.x = x;
            this.y = y;
            this.radius = rad;
            this.distance = dist;
        }

        public void calculateDistance(Pose2d robotPose, double robotHeight, double robotWidth) {
            double robotX = robotPose.getX();
            double robotY = robotPose.getY();
            this.distance = Math.min(Math.min(Math.min(
                    this.distanceLineSegment(new Vector2d(robotX, robotY),
                            new Vector2d(robotX + robotWidth, robotY)
                    ),
                    this.distanceLineSegment(new Vector2d(robotX + robotWidth, robotY),
                            new Vector2d(robotX + robotWidth,robotY + robotHeight)
                    )),
                    this.distanceLineSegment(new Vector2d(robotX, robotY),
                            new Vector2d(robotX,robotY + robotHeight)
                    )),
                    this.distanceLineSegment(new Vector2d(robotX,robotY + robotHeight),
                            new Vector2d(robotX + robotWidth,robotY + robotHeight)
                    )
            );
        }

        private double distanceLineSegment(Vector2d point1, Vector2d point2) {
            double px = point2.getX() - point1.getX();
            double py = point2.getY() - point1.getY();
            double temp = (px * px) + (py * py);
            double u = ((this.x - point1.getX()) * px + (this.y - point1.getY()) * py) / temp;
            if (u > 1) {
                u = 1;
            } else if (u < 0) {
                u = 0;
            }
            double x = point1.getX() + u * px;
            double y = point2.getY() + u * py;

            double dx = x - this.x;
            double dy = y - this.y;
            double distance = Math.sqrt(dx * dx + dy * dy) - this.radius;
            if (distance < 1) {
                distance = 1;
            }

            return distance;
        }
    }

    public GridGuider(double robotHeight, double robotWidth) {
        this.robotHeight = robotHeight;
        this.robotWidth = robotWidth;

        for (int i = -2; i < 3; i++) {
            for (int j = -2; j < 3; j++) {
                junctions.add(new Junction(24 * i, 24 * j, 3, 0));
            }
        }
    }

    public Vector2d calculateRepulsedVector(Pose2d robotPose) {
        Vector2d vector = new Vector2d();

        for (Junction j : junctions) {
            j.calculateDistance(robotPose, robotHeight, robotWidth);

            Vector2d dir = new Vector2d(j.x - robotPose.getX(), j.y - robotPose.getY());
            double dcubed = Math.pow(j.distance, 3);
            double strength = -G / dcubed;
            dir.times(strength);

            vector.plus(dir);
        }

        return vector;
    }
}
