package darp_pack;

public class Testing_Main {
    public static int[][] grid;
    public static int[][] A;
        
    public static void main(String[] args) {
        Main m = new Main();
        m.setRows(6);
        m.setCols(5);
        m.setMaxIter(1000);
        m.setDcells(30);
        m.setCC(0.01);
        m.setRL(0.0001);
        grid = new int[6][5];
        grid[0][4] = 2;
        grid[4][3] = 2;
        grid[3][0] = 2;
        grid[2][2] = 1;
        grid[2][1] = 1;
        grid[4][0] = 1;
        grid[2][3] = 1;
        grid[2][0] = 1;

        m.setGird(grid);
        m.setImp(false);
        A = m.darp_func();
        System.out.println("hi");
    }
}