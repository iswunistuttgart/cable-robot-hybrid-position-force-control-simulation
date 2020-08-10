function y = fibonacci(n)
%FIBONACCI Fibonacci numbers.
%
%    FIBONACCI(N) returns the Nth Fibonacci number.
%
%    The Fibonacci numbers satisfy the relations
%
%        Fib(N) = Fib(N-1) + Fib(N-2)    with    Fib(1) = 1, Fib(0) = 0
%
%        Fib(N-1)/Fib(N) -> (sqrt(5)-1)/2    as    N -> infinity

%    Author:        Peter J. Acklam
%    Time-stamp:  2004-02-09 17:48:19 +0100
%    E-mail:        pjacklam@online.no
%    URL:            http://home.online.no/~pjacklam

% Check number of input arguments.
narginchk(1, 1);

% Use explicit function.
y = sqrt(1 / 5) * (((1 + sqrt(5)) / 2).^n - ((1 - sqrt(5)) / 2).^n);

% Integer input should give integer output.
k = (n == round(n));
y(k) = round(y(k));

end