clear;
K1=5
K3=5
K2=50
M1=100 
M2=100
F1=100 
F2=100

B1 = tf ([K2],1)
B2 = tf (1,[K1])
B3 = tf ([F1,0],1)
B4 = tf ([M1,0,0],1)
B5 = tf ([K2],1)

B6 = tf ([-K3],1)
B7 = tf ([K2],1)
B8 = tf (1,[M2,0,0])
B9 = tf ([F2,0],1)
B10 = tf ([K2],1)

BlockMat = append(B1,B2,B3,B4,B5,B6,B7,B8,B9,B10)

connect_map = [1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
               2, -1, -3, -4, 5, 0, 0, 0, 0, 0, 0;...
               3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
               4, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
               5, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
               6, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
               7, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
               8, 6, -7, -9, 10, 0, 0, 0, 0, 0, 0;...
               9, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
               10, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0]


input_loc = 2
output_loc = [2,8]

sys = connect( BlockMat,connect_map,input_loc ,output_loc)

p = stepplot (sys)

setoptions(p,'RiseTimeLimits' ,[0,1])

tf = tf(sys)
figure();

pzmap(tf(1))
[wn,z ] = damp(sys)
figure();

pzmap(tf(2))
[wn1,z1 ] = damp(sys)
figure();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3. For any of the two transfer functions study the stability of the system..
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% To study the stability of the system we see the poles of the two transfer functions
% , we found that all poles are in -ve quarter and there are any poles in +ve quarter
% for both transfer functions, so the system is stable.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 4. If a fixed input force of 1N is applied to the system...
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% We already plot step responses for two transfer functions and 
% then we calculate the steady state values of these signals. 

info = stepinfo(tf(2));
RiseTime = info.RiseTime
PeakTime = info.PeakTime
MaxPeak = info.Peak
SettlingTime = info.SettlingTime
ess = abs(1 - info.SettlingMin)

info = stepinfo(tf(1));
RiseTime = info.RiseTime
PeakTime = info.PeakTime
MaxPeak = info.Peak
SettlingTime = info.SettlingTime
ess = abs(1 - info.SettlingMin)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 5. Suggest a modification to the system such that...
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% later we want to change B11 to tf(2)

clear;
B11 = tf(0.005, [1 ,2 ,2.1 ,1.1 ,0.0525])

BlockMat2 = B11

connect_map2 = [1, -1]
              
input_loc2 = 1
output_loc2 = 1

sys2 = connect( BlockMat2,connect_map2,input_loc2 ,output_loc2)

p2 = stepplot (sys2)

setoptions(p2,'RiseTimeLimits' ,[0,1])

tf2 = tf(sys2)
figure();

pzmap(tf2)
[wn22,z22 ] = damp(sys2)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 6. Simulate the system for a desired level (Xd) of 2 m. showing the response of X2.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
input = 2
p26 = stepplot (input*sys2)

setoptions(p26,'RiseTimeLimits' ,[0,1])

tf26 = tf(input*sys2)
figure();

pzmap(tf26)
[wn226,z226 ] = damp(input*sys2)
figure();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 7. For the response of X2 calculate the value of the rise time, peak time..
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[y,t]=step(input*sys2); 

info = stepinfo(input*sys2);
RiseTime = info.RiseTime
PeakTime = info.PeakTime
MaxPeak = info.Peak
SettlingTime = info.SettlingTime
ess = abs(input - y(end))


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 8. As a solution to reduce the value of ess a proportional controller...
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 kp = [1, 10, 100, 1000]
RiseTimeArr =  zeros(1, 4);
PeakTimeArr = zeros(1, 4);
MaxPeakArr = zeros(1, 4);
SettlingTimeArr = zeros(1, 4);
essArr = zeros(1, 4);

for i = 1:4
    B12 = kp(i) * B11

    sys3 = connect( B12,connect_map2,input_loc2 ,output_loc2)
    p3 = stepplot (input*sys3)
    setoptions(p3,'RiseTimeLimits' ,[0,1])
    
    tf3 = tf(input*sys3)
    figure();
    
    pzmap(tf3)
    [wn3,z3 ] = damp(input*sys3)
    figure();

    info3 = stepinfo(input*sys3);
    [y,t]=step(input*sys3); 
    
    RiseTimeArr(i) = info3.RiseTime
    PeakTimeArr(i) = info3.PeakTime
    MaxPeakArr(i) = info3.Peak
    SettlingTimeArr(i) = info3.SettlingTime
    essArr(i) = abs(input - y(end))


end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 9. If the desired displacement of the second mass is to be 4 m...
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% By hand calculations, when Kp > 4189.5, then we can get ess < 0.01, 
% but when we try it in Matlab, we see that the system becomes unstable,
% so for k > 4189.5 , the system becomes unstable and then the ess is 
% infinity, and it will be greater than 0.01 so we can’t do that with
% only a proportional controller.

kp = 4190
B122 = kp * B11
input = 4

sys3 = connect( B122,connect_map2,input_loc2 ,output_loc2)

p3 = stepplot (input*sys3)

setoptions(p3,'RiseTimeLimits' ,[0,1])

tf3 = tf(input*sys3)
figure();

pzmap(tf3)
[wn3,z3 ] = damp(input*sys3)
figure();
info3 = stepinfo(input*sys3);

ess = abs(input - info3.SettlingMin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 10. Suggest a suitable controller to eliminate ess...
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
input = 4
kp = 100
ki = 5

B13 = tf([kp, ki], [1, 0])
B14 = B13*B11

sys4 = connect( B14,connect_map2,input_loc2 ,output_loc2)
    
p4 = stepplot (input*sys4)
    
setoptions(p4,'RiseTimeLimits' ,[0,1])
    
tf4 = tf(input*sys4)
figure();
    
pzmap(tf4)
[wn4,z4 ] = damp(input*sys4)
[y,t]=step(input*sys4); 

info = stepinfo(input*sys4);
RiseTime = info.RiseTime
PeakTime = info.PeakTime
MaxPeak = info.Peak
SettlingTime = info.SettlingTime
ess=abs(input-y(end))


