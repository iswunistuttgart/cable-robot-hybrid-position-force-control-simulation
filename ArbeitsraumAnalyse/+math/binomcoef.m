function c = binomcoef(n, k)
%BINOMCOEF Binomial coefficient.
%
%    C = BINOMCOEF(N, K) returns the binomical coefficient N! / (K! * (N-K)!)
%    which, among other things, is the number of distinct ways to sample K
%    elements from a set of N elements when the order is irrelevant.  When not
%    both N and K are integers, GAMMA(N+1) / (GAMMA(K+1) * GAMMA(N-K+1)) is
%    returned.
%
%    C = BINOMCOEF(N) returns the same as BINOMCOEF(N, 0:N).  In this case N
%    must be a non-negative integer.

%    Author:        Peter J. Acklam
%    Time-stamp:  2004-02-09 15:27:32 +0100
%    E-mail:        pjacklam@online.no
%    URL:            http://home.online.no/~pjacklam

    % Check number of input arguments.
narginchk(1, 2);

if nargin == 1

    % Check array class.
    if ~isa(n, 'double')
        error('N must be double.');
    end

    % Check array size.
    if any((size(n) ~= 1))
        error('N must be scalar.');
    end

    % Check array values.
    if ~isreal(n) || (n < 0) || (n ~= round(n))
        error('N must be a real non-negative integer');
    end

    c = ones(1, n+1);
    m1 = floor(  n    / 2);
    m2 = floor((n-1) / 2);
    k = 1 : m1;
    c(2 : m1+1) = round(cumprod((n+1-k) ./ k));
    c(m1+2 : n+1) = c(m2+1 : -1 : 1);

else

    % Check array classes.
    if ~isa(n, 'double') || ~isa(k, 'double')
        error('N and K must be double.');
    end

    % Check array sizes.
    sn = size(n);
    sk = size(k);
    if any(sn ~= 1) && any(sk ~= 1) && ~isequal(sn, sk)
        error('Non-scalar arguments must have the same size.');
    end

    % Check array values.
    if ~isreal(n) || (n < 0)
        error('N must contain only real non-negative values.');
    end
    if ~isreal(k) || (k < 0)
        error('K must contain only real non-negative integer.');
    end
    if any(k(:) > n(:))
        error('All pairs (N, K) must satisfy 0 <= K <= N.');
    end

    % Without the following line BINOMCOEF(48, 25) would be wrong.
    k = min(k, n - k);
    c = exp(gammaln(n+1) - gammaln(k+1) - gammaln(n-k+1));

    % Integer input should give integer output.
    i = (n == round(n)) & (k == round(k));
    c(i) = round(c(i));

end

end