function mat = skew(v)
[r,c] = size(v);
if r ~= 3 || c ~=1
    error('Input is not a 3x1 vector')
else
    mat = [0, -v(3), v(2);...
           v(3), 0, -v(1);...
           -v(2), v(1), 0];
end
