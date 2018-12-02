format long
lr = 0.652;
fb = 0.650;
r = 0.192;
alpha = [atan2(lr, fb);atan2(lr, -fb); atan2(-lr, -fb); atan2(-lr, fb)]
beta = [pi/2 ; pi/2; 3*pi/2; 3*pi/2]-alpha
gam = [-pi/4; pi/4; -pi/4; pi/4]

A=[sin(alpha + beta+gam)./cos(gam), -cos(alpha+beta+gam)./cos(gam), -1*sqrt((lr/2)^2 + (fb/2)^2)*cos(beta + gam)./cos(gam)]/r

B = pinv(A)

%for test

% test = [1.30288; 1.30288; 1.30288; 1.30288];
% test = [-1.30288; -1.30288; -1.30288; -1.30288];
% test = [-1.30288; 1.30288; -1.30288; 1.30288];
 test = [1.30288; -1.30288; 1.30288; -1.30288];
% test = [-0.847656; -0.847656; 0.847656; 0.847656];
% test = [0.847656; 0.847656; -0.847656; -0.847656];
test(3) = test(3)*-1;
test(4) = test(4)*-1;
B*test