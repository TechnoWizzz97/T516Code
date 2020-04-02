function Rsq=Rsquared(yhat,y)
    %Fpredict and Fmeas are column vectors that represent the measured and
    %predicted data. The output argument Rsq is the R Squared value.
    yhat = yhat'; y = y';
    N=size(yhat,1);
    ybar=(1/N)*sum(y);
    SSerr=sum((y-yhat).^2);
    SStot=sum((y-ybar).^2);
    Rsq=1-(SSerr/SStot);
end
