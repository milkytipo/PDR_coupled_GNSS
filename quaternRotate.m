function v = quaternRotate(v, q)
%QUATERNPROD Rotates a vector by a quaternion
%
%   v = quaternRotate(v, q)
%
%   Rotates the 3D vector v by a quaternion q.
%
%   For more information see:
%   http://www.x-io.co.uk/node/8#quaternions
%
%	Date          Author          Notes
%	02/10/2011    SOH Madgwick    Initial release

    [row col] = size(v);
    v0XYZ = quaternProd(quaternProd(q, [zeros(row, 1) v]), quaternConj(q));
    v = v0XYZ(:, 2:4);
end

