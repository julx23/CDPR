function out=diff_w(input,w)
%     shift = [zeros(1,w) input];
%     out = -shift(w:end)+input(w:end);
    out = (input(w:end)-input(1:end-w+1))/w;
    out = [zeros(1,w) out];
end