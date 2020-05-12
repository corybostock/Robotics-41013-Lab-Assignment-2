close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear

startup_rvc;  
floorOffset = (-1.0896/2);                                                  % measured height or table in body0.ply
objectOffset = transl(0, 0 , -0.1);                                         % offset to lower endeffector onto object 
workspace = [-2.5 2.5 -2.5 2.5 (2*floorOffset) 1];