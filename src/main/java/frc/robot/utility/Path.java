// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

/** Add your docs here. */
public class Path {

    private PathPoint[] path;

    public Path(PathPoint[] points){
        path = points;
    }

    public PathPoint getPoint(int point){
        return path[point];
    }

}
