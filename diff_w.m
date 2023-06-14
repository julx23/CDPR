% differentiates with a window w to reduce noise
function out=diff_w(input,w)
    out = (input(w:end)-input(1:end-w+1))/w;
    out = [zeros(1,w) out];
end
