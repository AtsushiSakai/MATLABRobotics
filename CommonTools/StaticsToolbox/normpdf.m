function pdf=normpdf(x,mu,sigma)
% 正規分布の確率密度関数PDFを計算する関数
% StaticsToolBoxのnormpdf関数と同じ機能にしたつもり
% 参照:
% 正規分布 - MATLAB & Simulink - MathWorks 日本 
% http://jp.mathworks.com/help/stats/normal-distribution.html

if nargin==1
    mu=0;
    sigma=1;
elseif nargin==2
    sigma=1;
end

prefix=1/sqrt(2*pi)/sigma;
pdf=prefix.*exp(-(x-mu).^2./(2*sigma^2));

