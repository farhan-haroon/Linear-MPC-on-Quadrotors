function slBusOut = TwistWithCovariance(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    currentlength = length(slBusOut.Twist);
    for iter=1:currentlength
        slBusOut.Twist(iter) = bus_conv_fcns.ros.msgToBus.geometry_msgs.Twist(msgIn.Twist(iter),slBusOut(1).Twist(iter),varargin{:});
    end
    slBusOut.Twist = bus_conv_fcns.ros.msgToBus.geometry_msgs.Twist(msgIn.Twist,slBusOut(1).Twist,varargin{:});
                    currentlength = length(slBusOut.Covariance);
                    slBusOut.Covariance = double(msgIn.Covariance(1:currentlength));
end
