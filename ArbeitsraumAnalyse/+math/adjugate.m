function B = adjugate(A)
% ADJUGATE Calculate the adjugate matrix of A
%
% This finds the adjugate of square matrix A, and is valid even if A is singular
% or complex-valued. With U, S, and V obtained from [U,S,V] = SVD(A), it makes
% use of the identity adj(A) = det(U*V')*V*ADJ(S)*U', which holds even if A and
% S are both singular. The expression diag(prod(reshape(s0(ix),n-1,n),1)),
% accomplishes the evaluation of ADJ(S), each of whose diagonal elements is the
% product of all but one of the diagonal elements of S.
%
%   Inputs:
%
%   A                   NxN matrix of N at least 2.
%
%   Outputs:
%
%   B                   NxN matrix being the adjugate of A.
%
%   See also
%   SVD TOEPLITZ



%% File information
% Author: Roger Stafford - 10/18/06 https://mathworks.com/matlabcentral/profile/authors/870327-roger-stafford
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% See: https://mathworks.com/matlabcentral/fileexchange/12692-adjugate-adjugate-of-a-square-matrix
% Date: 2018-11-20
% Changelog:
%   2018-11-20
%       * Rename to `adjugate` as it really is the matrix adjugate rather than
%       the matrix adjoint
%   2018-11-17
%       * Initial release
%   2006-10-18
%       * Initial release on MATLAB FileExchange



%% Do your code magic here

[m,n] = size(A);

if (m ~= n) | (n < 2)
 error('Matrix A should be size n x n with n >= 2.')
end

[u,s,v] = svd(A);

s0 = diag(s);

ix = toeplitz(ones(n-1,1),[1 zeros(1,n-1)]) + repmat((1:n-1)',1,n);
   
B = det(u*v')*v*diag(prod(reshape(s0(ix),n-1,n),1))*u';


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header
