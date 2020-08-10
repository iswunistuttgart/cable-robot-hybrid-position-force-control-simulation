function n = sigdigits(x)
% SIGDIGITS Return significant digits in a numeric non-uint64 number
%
% SIGDIGITS(X) returns "significant" digits in a number, in the sense that if
% you printed any fewer digits the reverse conversion would not equal the
% original number. I.e., the value returned from this function is the minimum
% number of digits you must print in order to recover the original number with a
% reverse conversion.
%
%   Inputs:
%
%   X                   Description of argument X
%
%   Outputs:
%
%   N                   Description of argument N



%% File information
% Author: James Tursa
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% See: https://de.mathworks.com/matlabcentral/answers/142819-how-to-find-number-of-significant-figures-in-a-decimal-number
% Date: 2018-11-08
% Changelog:
%   2018-11-08
%       * Initial release



%% Do your code magic here

if( ~isnumeric(x) || ~isfinite(x) || isa(x,'uint64') )
    error('Need any finite numeric type except uint64');
end
if( x == 0 )
    n = 0;
    return;
end
x = abs(x);
y = num2str(x,'%25.20e'); % Print out enough digits for any double
z = [' ' y]; % Pad beginning to allow rounding spillover
n = find(z=='e') - 1; % Find the exponent start
e = n;
while( str2double(y) == str2double(z) ) % While our number is still equal to our rounded number
    zlast = z;
    c = z(e); % Least significant printed digit
    if( c == '.' )
        e = e - 1;
        c = z(e);
    end
    z(e) = '0'; % 0 the least significant printed digit
    e = e - 1;
    if( c >= '5' ) % Round up if necessary
        c = z(e);
        if( c == '.' )
            e = e - 1;
            c = z(e);
        end
        while( true ) % The actual rounding loop
            if( c == ' ' )
                z(e) = '1';
                break;
            elseif( c < '9' )
                z(e) = z(e) + 1;
                break;
            else
                z(e) = '0';
                e = e - 1;
                c = z(e);
                if( c == '.' )
                    e = e - 1;
                    c = z(e);
                end
            end
        end
    end
end
n = n - 1;
z = zlast(1:n); % Get rid of exponent
while( z(n) == '0' ) % Don't count trailing 0's
    n = n - 1;
end
n = n - 2; % Don't count initial blank and the decimal point.


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header
