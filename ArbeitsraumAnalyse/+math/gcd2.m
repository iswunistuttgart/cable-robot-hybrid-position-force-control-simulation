function g = gcd2(x)
%GCD2 Greatest common divisor of all elements.
%
%    GCD2(X) is the greatest common divisor of all elements in X.
%
%    See also GCD, LCM, LCMALL.

%    Author:        Peter J. Acklam
%    Time-stamp:  2004-02-09 18:31:56 +0100
%    E-mail:        pjacklam@online.no
%    URL:            http://home.online.no/~pjacklam

% Check number of input arguments.
narginchk(1, 1);

% Check array values.
if ~isreal(x)
    error('Argument must be a numeric and real array.');
end

% Any NaN or +/-Inf in input gives NaN output.
if any(~isfinite(x))
    g = NaN;
    return;
end

% Check the finite elements.
if ~isequal(x, round(x))
    error('All finite elements must be real integers.');
end

% Now find greatest common divisor.
n = numel(x);
g = 0;
for i = 1 : n
    g = gcd(g, x(i));
    if g == 1
        break
    end
end

end
