function skew = vec2skew(vec)%#codegen
% VEC2SKEW Turn the input into its skew-symmetrix matrix form
% 
%   SKEW = VEC2SKEW(VEC) turns vec into its skew-symmetric matrix form of type
%   [0,      -vec(3),  vec(2); ...
%    vec(3),  0,      -vec(1); ...
%   -vec(2),  vec(1),  0];
%   If VEC is a Mx3 matrix, then SKEW is a 3x3xM matrix of skew-symmetrix
%   matrices for each row of VEC concatenated along the third dimension
%   
%   
%   Inputs:
%   
%   VEC: Matrix of column size 3. If VEC is Mx3 matrix, output is a 3x3xM
%   matrix.
%
%   Outputs:
%   
%   SKEW: Skew-symmetric matrix of VEC. If VEC is a Mx3 matric, SKEW is 3x3xM.
%



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-04-13
% Changelog:
%   2016-04-13
%       * Allow vec2skew to work on matrices thanks to help from quat2rot.m
%       * Update help doc
%       * Add block "File information"
%   2015-04-24
%       * Initial release



%% Input parsing
% Gotta have doubles
assert(isa(vec, 'double') || isa(vec, 'sym'), 'Input must be double or symbolic.');
% Gotta have at least three elements inside VEC
assert(size(vec, 2) == 3, 'Input must have three columns.');



%% Process inputs
% Get all vectors for wich to get the skew-symmetric matrix
aVectors = transpose(vec);
% Number of vectors == columns of aVectors
nVectors = size(aVectors, 2);
% Reshape the vector in the depth dimension
aVectors2 = reshape(aVectors, [3, 1, nVectors]);

% Quicker access to important parts
s = zeros(1, 1, nVectors);
x = aVectors2(1,1,:);
y = aVectors2(2,1,:);
z = aVectors2(3,1,:);



%% Magic (copied from quat2rotm)

% Explicitly define concatenation dimension for codegen
tempS = cat(1, s, -z, y, z, s, -x, -y, x, s);
skew = reshape(tempS, [3, 3, nVectors]);
skew = permute(skew, [2, 1, 3]);


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header
