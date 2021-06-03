#!/bin/bash

# Code to compile all the necessary Java programs before running them later
# Compiling for DARP algorithm - gets run in Run_Java.bat
javac darp_pack/ConnectComponent.java
javac darp_pack/DARP.java
javac DARP_Java_New.java

# Compiling for Prim MST Algorithm - gets run in pMST_Run_Java.bat
javac STC_pack/prim.java
javac Running_STC.java