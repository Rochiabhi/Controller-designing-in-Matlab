%question 1: deisging of controller of a state space model  using pole placement technique 
% two meathods of desging controller Ackerman's formule and transformation approach

% meathod 1: desging of pole placement using transformation approach ie, given system is not in controllable cannonical form then first
% we need to convert the given system into the CCF format.
clc ;
close all;
clear all;

A =[-4 1 0;0 -3 1;0 0 -2];
B =[0; 0; 16];
C= [2 1 0];
D = 0;
disp('given system of eqution');
sys = ss(A,B,C,D);

%check controllablity 
M = [B A*B A^2*B];
px  = poly(A);
AZ = [0 1 0; 0 0 1;-24 -26 -9];
BZ = [0;0; 16];
MZ = [BZ AZ*BZ AZ^2*BZ];
MZd = det(MZ);
% deteminant of MZ is not eqaul to zero hence, given system is controllable

p = M*inv(MZ);

%our desired characterstics eqution is:
%s^3+9.709s^2+33.853s+69.58
 % coeffecient of desired characterstics eqaution.
 az0 = 69.58;az1 = 33.853;az2 =9.709;
 % coffecient of charactertics eqution of given system.
 a0 = px(4);a1 = px(3);a2 = px(2);
 % it is the value of K tranaformed system
 kz = [az0-a0 az1-a1 az2-a2]/16;
 % it is the value of k for our given system
 kx = (kz*inv(p));

 
% method 2 : finding the value of K using ackermas formulae for the same above problem: 
%our desired characterstics eqation is s^3+9.709s^2+33.853s+69.58
%our controllablity would remains same.
Qc = M;
%characterstics matrix associated with desired chractertsics equation
pA = A*A*A + 9.709*A*A + 33.853*A + 69.58*eye(3); 
%putting the value of different quantity the ackerman's fomulae.
kc = [0 0 1]*inv(Qc)*pA;




% question number 2
%disp('question number 2');
%(' design a state space observeber using transformation and ackermans formule');
%given : (' the unit step response of a plant has to have peak overshoot of less than 20% and settling time is 2sec');
A1=[-3 1 ;-2 0];
B1 = [1;2];
C1 = [1 0];
D1 = 0;
sys1 = ss(A1,B1, C1, D1);

%check observablity
Q = [C1' A1'*C1']';
Qd = det(Q);

%since determinant of Q is not eqaul to zero hence th given system is observable
%our desired characterstics equation is
%s^2+3.96s+19.36
Qx = poly(A1);
Kq = [3.96-Qx(2) 19.36-Qx(3)];

% method 2 for state space observeber design suing ackerman's formulae .
%disp('solving th same problem using ackermans formule for oserveber design');
%disp('our desired characterstics equation is ');
%disp('s^2+3.96s+19.36');
Qa = A1^2+ 3.96*A1 + 19.36*eye(2);
%putting the different quantity into ackerman's formulae for observeber design
Kqa = Qa*inv(Q)*[0;1];





















