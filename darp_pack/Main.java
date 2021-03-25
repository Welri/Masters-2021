package darp_pack;
// File handling stuff
import java.io.*;

public class Main {
    private int rows, cols, MaxIter, dcells;
    private boolean imp;
    private int[][] EnvironmentGrid;
    private double CCvariation;
    private double randomLevel;
    private int[][] A1;

    public void setRows(int r) {
        rows = r;
    }

    public void setCols(int c) {
        cols = c;
    }

    public void setMaxIter(int maxIter) {
        MaxIter = maxIter;
    }

    public void setDcells(int d) {
        dcells = d;
    }

    public void setImp(boolean Imp) {
        imp = Imp;
    }

    public void setGird(int[][] EG) {
        EnvironmentGrid = EG;
    }

    public void setCC(double cc) {
        CCvariation = cc;
    }

    public void setRL(double rl) {
        randomLevel = rl;
    }

    public int[][] darp_func() {
        DARP problem = new DARP(rows, cols, EnvironmentGrid, MaxIter, CCvariation, randomLevel, dcells, imp);
        problem.constructAssignmentM();
        A1 = problem.getAssignmentMatrix();
        discr_achieved = problem.getAchievedDiscr();
        return A1;
    }
}