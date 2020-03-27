function [torque1,torque2,torque3] = worstCaseTorque(arm,g)
% calculates the worst case of the torque (aka when all the arm's linkages
% are horizontal
payloadMass = 300; % kg 
payloadWeight = payloadMass*g;
payloadLength = 0.5; % m
% COM1 = (arm.weight1*arm.base + arm.weight2*arm.secondlink + arm.weight3*arm.endeffector + payloadWeight*payloadLength)/ ...
%     (arm.weight1+arm.weight2+arm.weight3+payloadWeight)
% COM2 = ((arm.weight2*arm.secondlink + arm.weightt*arm.endeffector + payloadWeight*payloadLength)/ ...
%     (arm.weight2+arm.weight3+payloadWeight))
% COM3 = ((arm.weight3*arm.endeffector + payloadWeight*payloadLength)/ ...
%     (arm.weight3+payloadWeight))
COM1 = (arm.base+arm.secondlink+arm.endeffector+payloadLength)/2;
COM2 = (arm.secondlink+arm.endeffector+payloadLength)/2;
COM3 = (arm.endeffector+payloadLength)/2;
% kN*m
torque1 = ((arm.weight1+arm.weight2+arm.weight3+payloadWeight)*COM1);
torque2 = ((arm.weight2+arm.weight3+payloadWeight)*COM2);
torque3 = ((arm.weight3+payloadWeight)*COM3);
end