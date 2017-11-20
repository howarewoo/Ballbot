%% Max necessary Torque
clc

C = .1;     % coefficient of friction
Wb = 3;     % weight of ball in lbs
Wr = 10;    % weight of robot in lbs
W = Wb + Wr % total weight
D = 5;      % diameter of wheels in inches

T = 8*C*W*D

%% Contact Radius on Ball (rounded to nearest 1/16")
clc

V = 5*cosd(45);     % radius
N = 1/16;           % rounding multiple
R = N*round(V/N)    % radius rounded