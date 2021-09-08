@ECHO OFF

@REM Code to compile all the necessary Java programs before running them later
@REM Compiling for DARP algorithm - gets run in Run_Java.bat
javac darp_pack/ConnectComponent.java
javac darp_pack/DARP.java
javac DARP_Java_New.java
@REM For Astar files
javac darp_pack/AStar.java
javac darp_pack/Node.java

@REM Compiling for Prim MST Algorithm - gets run in pMST_Run_Java.bat
javac STC_pack/prim.java
javac Running_pMST.java