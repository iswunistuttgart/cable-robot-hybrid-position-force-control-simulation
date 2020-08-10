function isp = isprime2(X)
%ISPRIME2 True for prime numbers.
%
%    ISPRIME2(X) is 1 for the elements of X which are prime, 0 otherwise.
%
%    See also FACTOR, PRIMES, ISPRIME.

%    Author:        Peter J. Acklam
%    Time-stamp:  2003-10-13 15:11:12 +0200
%    E-mail:        pjacklam@online.no
%    URL:            http://home.online.no/~pjacklam

% check number of input arguments
narginchk(1, 1);

if isempty(X)
    isp = logical(zeros(size(X)));
    return
end

isp = logical(X);
n = max(X(:));
maxfact = floor( sqrt(n) );                  % Maximum possible factor.
p = primes( maxfact );                         % All possible factors.
for k = 1:numel(isp)
    if X(k) <= maxfact
        isp(k) = any( X(k) == p );            % Is it among the primes?
    else
        isp(k) = all( rem( X(k), p ) );     % Is it divisible by any prime?
    end
end

end