import darp_pack.*;

public class DARP_Java_New{
    public int rows, cols, MaxIter, dcells,i,j;
    public boolean imp;
    public int[][] EnvironmentGrid,A;
    public double CCvariation;
    public double randomLevel;
    public static void main(String[] args){
        DARP_Java_New t = new DARP_Java_New();
        t.rows = 6;
        t.cols = 6;
        t.MaxIter = 1000;
        t.dcells = 30;
        t.CCvariation = 0.001;
        t.randomLevel = 0.0001;
        t.imp = false;
        t.EnvironmentGrid = new int[][]{{0,0,0,2,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,2,0,0},{0,0,0,0,0,0}};
     
        DARP problem = new DARP(t.rows, t.cols, t.EnvironmentGrid, t.MaxIter, t.CCvariation, t.randomLevel, t.dcells, t.imp);
        problem.constructAssignmentM();
        t.A = problem.getAssignmentMatrix();
    }
}