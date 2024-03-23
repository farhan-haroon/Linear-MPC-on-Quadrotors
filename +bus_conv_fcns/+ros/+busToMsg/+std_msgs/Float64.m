function rosmsgOut = Float64(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.Data = double(slBusIn.Data);
end
