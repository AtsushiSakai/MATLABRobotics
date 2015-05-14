function cdf=normcdf(x,mu,sigma)
%正規累積分布関数CDFと計算する関数
%StatticsToolBoxのnormcdf関数と同じ機能にしたつもり
%参照:
%正規累積分布関数 - MATLAB normcdf - MathWorks 日本 
%http://jp.mathworks.com/help/stats/normcdf.html
if nargin==1
    mu=0;
    sigma=1;
elseif nargin==2
    sigma=1;
end
cdf=[];
resolution=10000;
for i=1:length(x)
    xt = 0 : (x(i) / resolution) : x(i);
    cdf= [cdf sum(normpdf(xt,mu,sigma)* x(i)/resolution)];
end