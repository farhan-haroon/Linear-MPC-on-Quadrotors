function rosmsgOut = PositionTarget(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.Header = bus_conv_fcns.ros.busToMsg.std_msgs.Header(slBusIn.Header,rosmsgOut.Header(1));
    rosmsgOut.CoordinateFrame = uint8(slBusIn.CoordinateFrame);
    rosmsgOut.TypeMask = uint16(slBusIn.TypeMask);
    rosmsgOut.Position = bus_conv_fcns.ros.busToMsg.geometry_msgs.Point(slBusIn.Position,rosmsgOut.Position(1));
    rosmsgOut.Velocity = bus_conv_fcns.ros.busToMsg.geometry_msgs.Vector3(slBusIn.Velocity,rosmsgOut.Velocity(1));
    rosmsgOut.AccelerationOrForce = bus_conv_fcns.ros.busToMsg.geometry_msgs.Vector3(slBusIn.AccelerationOrForce,rosmsgOut.AccelerationOrForce(1));
    rosmsgOut.Yaw = single(slBusIn.Yaw);
    rosmsgOut.YawRate = single(slBusIn.YawRate);
end
