function slBusOut = PoseWithCovariance(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    currentlength = length(slBusOut.Pose);
    for iter=1:currentlength
        slBusOut.Pose(iter) = bus_conv_fcns.ros.msgToBus.geometry_msgs.Pose(msgIn.Pose(iter),slBusOut(1).Pose(iter),varargin{:});
    end
    slBusOut.Pose = bus_conv_fcns.ros.msgToBus.geometry_msgs.Pose(msgIn.Pose,slBusOut(1).Pose,varargin{:});
                    currentlength = length(slBusOut.Covariance);
                    slBusOut.Covariance = double(msgIn.Covariance(1:currentlength));
end
