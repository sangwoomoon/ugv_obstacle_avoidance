

% ASSUMPTIONS.

% RIGID BODY.
% USE SYMMETRIC INTERTIA TENSOR.
% EACH WHEELS ARE RIGID BODIES WITHOUT
%  ONLY INTEGRATE ANGULAR VELOCITIES ABOUT AXLE.
% SUSPENSION IS MODDELLED USING SPRINGS.
% GROUND FORCES ARE APPLIED
% DRIVE TRAIN IS ALSO MODELLED


Tire_nat = 0.6091;  % Natural Tire diameter.
Mass = 1285;     % Mass of Car.
tensor_m(1) = Mass/50;  % Mass Tensor
tensor_i(1) = tensor_m(1)*Tire_nat^2/2;  % inertia tensor.
sus_k = (9.801*Mass/4)/(Tire_nat/20);    % suspension modeling. 5% under normal weight
