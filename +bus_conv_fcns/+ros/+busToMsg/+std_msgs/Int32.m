function rosmsgOut = Int32(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.Data = int32(slBusIn.Data);
end
