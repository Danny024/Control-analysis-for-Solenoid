clear;
%Resistance of Solenoid
R = 225;
%Inductance of Solenoid
L = 0.99
%Resistance of amplifier
R_1 = 50 %0.5,5,50
s = tf ('s')
%PID Parameters
kp = 10;
ki = 10;
kd = 5;
%Define PID controller
C_s = kp + (ki/s) + (kd*s)
%Unity Feedback
H_s = 1
%Transfer Function of Solenoid
G_s = 1/(R+s*L)
%Transfer Function of Amplifier
G_a = R_1
%Overall Transfer Function of System
G_total = (G_s*C_s*G_a)/(1+(G_s*C_s*G_a*H_s))
[num, den] = tfdata(G_total,'v')
[A,B,C,D] = tf2ss(num,den)
%Observability Matrix
OB = obsv(A,C)
Rank_ob = rank (OB)
%Controllability Matrix
CO = ctrb (A,B)
Rank_co = rank(CO)
step (num,den)
%Check for stability
Stability = istable (G_total)
pzmap (G_total)
controlSystemDesigner(G_total)