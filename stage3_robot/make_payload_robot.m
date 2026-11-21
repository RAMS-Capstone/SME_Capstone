function rb = make_payload_robot(robot, mass)
% MAKE_PAYLOAD_ROBOT  Copy of the CTC robot model with the ball as a payload.
%   The plant carries a 0.1 kg ball welded to the EndEffector_Link origin
%   (Subsystem/Inertia: mass 0.1 kg, inertia 1.6e-5 kg m^2 = 2/5*m*r^2 with r = 2 cm).
%   mass : payload mass the CONTROLLER assumes [kg]. Use 0.1 for the true value, other values
%          to study a wrong payload setting. Inertia is scaled with mass (same 2 cm sphere).
rb = copy(robot);
ball = rigidBody('payload_ball');
ball.Mass = mass;
ball.CenterOfMass = [0 0 0];
Iball = 1.6e-4*mass;                       % = 1.6e-5 kg m^2 at 0.1 kg
ball.Inertia = [Iball Iball Iball 0 0 0];  % [Ixx Iyy Izz Iyz Ixz Ixy]
ball.Joint = rigidBodyJoint('payload_weld','fixed');
ee = 'EndEffector_Link';
if ~any(strcmp(rb.BodyNames, ee)), ee = rb.BodyNames{end}; end
addBody(rb, ball, ee);
end
