function [x, w] = gausslegendre(n, l, u)
% GAUSSLEGENDRE Determine Gauss-Legendre Quadrature abscissae and weights
%
%   [X, W] = GAUSSLEGENDRE(N) calculates quadrature abscissae values X and
%   weights W for a Gauss-Legendre quadrature using N points. The quadrature
%   interval will be set to [-1, 1].
%
%   [X, W] = GAUSSLEGENDRE(N, L) transforms the abscissae values and weights
%   such that they apply to the new interval [L, 1]. L is a 1xK array of lower
%   boundary values.
%
%   [X, W] = GAUSSLEGENDRE(N, L, U) transforms the abscissae values and weights
%   such that they apply to the new interval [L, U]. L and U are 1xK array of
%   lower and upper boundary values, respectively.
%
%   Inputs:
%
%   N                   Number of quadrature points to use.
%
%   L                   1xK array of lower bounds.
%
%   U                   1xK array of upper bounds.
%
%   Outputs:
%
%   X                   NxK array of abscissae values for all intervals.
%
%   W                   NxK array of function weights for all intervals and
%                       abscissae values.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2018-11-08
% Changelog:
%   2018-11-08
%       * Initial release



%% Validate attributes
try
  narginchk(1, 3);
  
  nargoutchk(0, 2);
  
  % GAUSSLEGENDRE(X)
  if nargin < 2
    l = -1;
  end
  
  % GAUSSLEGENDRE(X, L)
  if nargin < 3
    u = ones(size(l));
  end
  
  
  % GAUSSLEGENDRE(X, L, U)
  validateattributes(n, {'numeric'}, {'scalar', 'nonempty', 'positive', 'nonnan', 'finite', 'nonsparse'}, mfilename, 'N');
  validateattributes(l, {'numeric'}, {'row', 'nonempty', 'nonnan', 'finite', 'nonsparse'}, mfilename, 'L');
  validateattributes(u, {'numeric'}, {'row', 'nonempty', 'nonnan', 'finite', 'nonsparse'}, mfilename, 'U');
  assert(all(l < u), 'Upper boundaries must be larger than lower boundaries');
  
catch me
  me.throwAsCaller()
end



%% Do your code magic here
% Number of boundaries i.e., how many polynomials to solve
nb = numel(l);

% % Init variables
% x = zeros(n, nb);
% w = zeros(n, nb);

% Width of intervals
h = u - l;

% First, we will calculate the abscissae and weights for the standard [-1, 1]
% interval and for one function only. Afterwards, we will transform the
% abscissae and weights

% Holds the [-1, 1] abscissae values and weights
x = zeros(n, 1);
w = zeros(n, 1);

% How many polynomials
m = (n + 1)/2;

% Loop over polynomial
for im = 1:m
  % Initial estimate
    z = cos( pi * (im - 0.25) / (n + 0.5) );
    z1 = z + 1;
    
    % Solve Legendre polynomial
    while abs(z - z1) > eps
        p1 = 1;
        p2 = 0;
        
        for in = 1:n
            p3 = p2;
            p2 = p1;
            % The actual Legendre polynomial
            p1 = ((2*in - 1) * z * p2 - (in - 1) * p3) / in;
        end
        
        % Derivative of the Legendre polynomial
        pp = n * (z * p1 - p2) / (z^2 - 1);
        z1 = z;
        z = z1 - p1 / pp;
    end
    
    % Build symmetric abscissaes
    x(im) = z;
    x(n + 1 - im) = -z;
    % Build symmetric weights
    w(im) = 1 / ( (1 - z^2) * pp^2 );
    w(n + 1 - im) = w(im);
end

% Now that we have the default abscissae values and weights, we will scale them
% up to the [L, U] interval
% Shift abscissae
x = ( l .* (1 - x) + u .* (1 + x) ) / 2;
% Scale weights
w = ( u - l ) .* w;


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header
