function p = factorial2(n)
%FACTORIAL2 Factorial function.
%
%    FACTORIAL2(N) = PROD(1:N) = 1*2*...*N.  NaN is returned if N is negative or
%    not an integer.  The output overflows (becomes Inf) for N > 170.
%
%    See also PROD, GAMMA.

%    Author:        Peter J. Acklam
%    Time-stamp:  2004-02-09 17:45:45 +0100
%    E-mail:        pjacklam@online.no
%    URL:            http://home.online.no/~pjacklam

% Check number of input arguments.
narginchk(1, 1);

% Check array class.
if ~isa(n, 'double')
    error('Input must be a double array.');
end

% Check array values.
if ~isreal(n) || any(n(:) < 0) || any(n(:) ~= round(n(:)))
    error('Input must be an array of real non-negative integers.');
end

% The `maxval' value is the largest value `n' for which prod(1:n) < Inf.  This
% value is 34 with IEEE single precision, 170 with IEEE double precision,
% and 6936 with IEEE extended double precision.

maxval = 170;                     % prod(1:maxval+1) = Inf
table  = cumprod(1 : min(max(n(:)), maxval));

% Initialize output.
p = NaN(size(n));

% Table lookup.
k = (n == round(n)) & (1 <= n) & (n <= maxval);
p(k) = table(n(k));

% Overflow.
k = (n == round(n)) & (n > maxval);
p(k) = Inf;

% Special case n = 0.
k = (n == 0);
p(k) = 1;

end