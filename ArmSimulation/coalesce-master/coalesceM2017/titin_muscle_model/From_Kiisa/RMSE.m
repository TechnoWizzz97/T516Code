function r=RMSE(estimate,observed)
% Function to calculate root mean square error from a observed vector or matrix
% and the corresponding estimates.
% Usage: r=rmse(observed,estimate)
% Note: observed and estimates have to be of same size
% Example: r=rmse(randn(100,100),randn(100,100));

% delete records with NaNs in both observedsets first
I = ~isnan(observed) & ~isnan(estimate);
observed = observed(I);
estimate = estimate(I);

r=sqrt( sum((observed(:)-estimate(:)).^2) / numel(observed));

end
