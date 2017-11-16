function [output] = senserror(input,sensornoise)

a = input - input*sensornoise/2;
b = input + input*sensornoise/2;

output = (b - a)*rand() + a;

end