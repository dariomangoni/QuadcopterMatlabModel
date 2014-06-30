function [output] = low_pass_filter( input , dt , freq_taglio)
d=fdesign.lowpass('Fp,Fst,Ap,Ast',freq_taglio,freq_taglio+10,1,60,1/dt);
Hd = design(d,'equiripple');
output = filter(Hd,input);