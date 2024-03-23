function slBusOut = Pose(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    slBusOut.X = single(msgIn.X);
    slBusOut.Y = single(msgIn.Y);
    slBusOut.Theta = single(msgIn.Theta);
    slBusOut.LinearVelocity = single(msgIn.LinearVelocity);
    slBusOut.AngularVelocity = single(msgIn.AngularVelocity);
end
