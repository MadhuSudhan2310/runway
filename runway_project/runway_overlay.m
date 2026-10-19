function rgb = runway_overlay(img, z, ok, xhat, lock)
%RUNWAY_OVERLAY  Draw detections (red) and tracked edges (green) on the frame.
%   rgb = runway_overlay(img, z, ok, xhat, lock)  ->  H x W x 3 uint8
%   Top-left square: green = tracker locked, red = not locked.
%#codegen
p = runway_params();
[H, ~] = size(img);
g8  = uint8(255*min(max(img, 0), 1));
rgb = cat(3, g8, g8, g8);
yt  = p.roiTop;

for y = yt:H
    a = (y - yt)/(H - yt);                       % 0 at ROI top, 1 at bottom
    for s = 1:2
        i1 = 2*s - 1;                            % z(i1) = bottom, z(i1+1) = top
        if ok(s) > 0.5
            x = round(z(i1+1) + a*(z(i1) - z(i1+1)));
            rgb = put_px(rgb, x, y, uint8([255 0 0]));
        end
        x = round(xhat(i1+1) + a*(xhat(i1) - xhat(i1+1)));
        rgb = put_px(rgb, x,   y, uint8([0 255 0]));
        rgb = put_px(rgb, x+1, y, uint8([0 255 0]));
    end
end

if lock > 0.5
    c = uint8([0 200 0]);
else
    c = uint8([220 0 0]);
end
for yy = 3:8
    for xx = 3:8
        rgb = put_px(rgb, xx, yy, c);
    end
end
end

function rgb = put_px(rgb, x, y, c)
[H, W, ~] = size(rgb);
if x >= 1 && x <= W && y >= 1 && y <= H
    rgb(y, x, 1) = c(1);
    rgb(y, x, 2) = c(2);
    rgb(y, x, 3) = c(3);
end
end
