offset = [-0.08; -0.12];
test_measure = [0; 0.86];
position = [0; 0];
psi = 0-pi/2;
global_m = ([cos(psi), -sin(psi); ...
            sin(psi), cos(psi)] * (test_measure(:) + offset)) + position
        
test_measure = [-0.64; 0.06];
position = [0; 0];
psi = -pi/2-pi/2;
global_m = ([cos(psi), -sin(psi); ...
            sin(psi), cos(psi)] * (test_measure(:) + offset)) + position
        
test_measure = [0.14; -0.61];
position = [0; 0];
psi = pi-pi/2;
global_m = ([cos(psi), -sin(psi); ...
            sin(psi), cos(psi)] * (test_measure(:) + offset)) + position
        
test_measure = [0.84; 0.17];
position = [0; 0];
psi = pi/2-pi/2;
global_m = ([cos(psi), -sin(psi); ...
            sin(psi), cos(psi)] * (test_measure(:) + offset)) + position