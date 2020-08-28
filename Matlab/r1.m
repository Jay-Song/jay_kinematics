clc; clear all;
syms c1 s1 c2 s2 c3 s3 c4 s4 c5 s5 c6 s6 c8 s8;

R1 = [c1 -s1 0;
      s1  c1 0;
       0   0 1];
R2 = [1  0   0
      0 c2 -s2;
      0 s2  c2];
R3 = [c3 0 s3;
       0 1  0;
     -s3 0 c3];
R4 = [c4 0 s4;
       0 1 0;
     -s4 0 c4];
R5 = [c5 0 s5
       0 1  0;
     -s5 0 c5];
R6 = [1  0   0
      0 c6 -s6;
      0 s6  c6];
R8 = [c8 0 s8;
       0 1 0;
     -s8 0 c8];


% (R1*R2*R8)*R6
R2*R3

% R1*R2*R8