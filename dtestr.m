function ds = dtestr(t,s)
global eta K;
ds = -eta*tanh(s) - K*(s);

end