function W = quat2ratem(q)%#codegen
% QUAT2RATEM Gives the quaternion rate matrices
% 
%   T = QUAT2RATEM(Q) determines the quaternion rate matrices of size 3x4xN for
%   each quaternion in Q. Quaternion rate matrix T can then be used to be
%   post-multiplied by the quaternion to get the angular velocity.
%
%   Inputs:
%   
%   Q               Nx4 matrix of quaternions to get rate matrice from.
%
%   Outputs:
%
%   T               3x4xN matrix of quaternion rate matrix.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-10-19
% Changelog:
%   2016-10-19
%       * Add check for `numeric` and `sym` argument types
%       * Add support for symbolic variables
%   2016-05-10
%       * Add codegen-directive
%   2016-04-29
%       * Change name to ```quat2ratem```
%   2016-04-22
%       * Initial release



%% Assertion
% Input must be of type double or symbolic
assert(isa(q, 'numeric') || isa(q, 'sym'), 'PHILIPPTEMPEL:MATLAB_TOOLING:MATH:QUAT2RATEM:InvalidType', 'Input must be numeric or symbolic');
% Input must be of size Nx4
assert(size(q, 2) == 4, 'PHILIPPTEMPEL:MATLAB_TOOLING:MATH:QUAT2RATEM:InvalidSize', 'Number of columns must be qual to 4');



%% Process inputs
% Get matrix of quaternion
aQuaternions = q;
% Number of quaternions
nQuaternions = size(aQuaternions, 1);
% Normalize quaternions if not symbolic i.e., each row of the matrix of quaternions
if isa(aQuaternions, 'numeric')
    aQuaternions = mnormrow(aQuaternions);
end
aQuaternions = transpose(aQuaternions);



%% Magic, here we go
% Reshape the quaternions in the depth dimension
aQuaternions2 = reshape(aQuaternions, [4, 1, nQuaternions]);

% Extract the parts [qw, qx, qy, qz] from q to make writing code easier
qw = aQuaternions2(1,1,:);
qx = aQuaternions2(2,1,:);
qy = aQuaternions2(3,1,:);
qz = aQuaternions2(4,1,:);

% Explicitly define concatenation dimension for codegen
tempT = cat(1 ...
    , -qx, qw, -qz, qy ...
    , -qy, qz, qw, -qx ...
    , -qz, -qy, qx, qw ...
);

% Reshape the matrix to its proper dimensions
aTrafos = reshape(tempT, [4, 3, nQuaternions]);
% Restore correct order of dimensions
aTrafos = permute(aTrafos, [2, 1, 3]);



%% Assign output quantities
% Only output is the matrix of transformation matrices
W = aTrafos;


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header
