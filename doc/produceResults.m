%% load data
 addpath(genpath('./libsvm-3.12/'));

rand('state', 0);
randn('state', 0);

clear all
close all

X= load('segmentMeasures.eig.txt');
y= load('segmentLabels.eig.txt');

X= bsxfun(@minus, X, mean(X)); % ensure normalized
X= bsxfun(@rdivide, X, std(X));

%%
allaps=[];
for times=1:1

% Randomly scramble X and y
%ri= randperm(size(X,1));
%X= X(ri, :);
%y= y(ri);

H= floor(size(X,1)/2);
Xtrain= X(1:H, :); ytrain= y(1:H, :);
Xtest= X(H+1:end, :); ytest= y(H+1:end, :);

posix=1; %because libsvm bases labels on first training label
if(ytrain(1)==-1), posix=2; end

%% Evaluate nearest neighbor scores

pd= pdist2(Xtest, Xtrain); % rows are testing instaces
[V1, IX1]= min(pd(:, ytrain==1), [], 2);
[V2, IX2]= min(pd(:, ytrain==-1), [], 2);
NNScores= V2-V1;
[prec, rec, ap]= computepr(ytest, NNScores, 'a', false);
ap

%% train model: kernel SVM

N= size(Xtrain, 1);
folds = 5;
[C,gamma] = meshgrid(-5:2:15, -15:2:3);
cv_acc = zeros(numel(C),1);
for i=1:numel(C)
    avap=0;
    for j = 1:folds
        ix = crossvalind('Kfold', N, folds);
        testix = (ix == j); trainix = ~testix;

        sopts= sprintf('-b 1 -c %f -g %f', 2^C(i), 2^gamma(i));
        model = svmtrain(ytrain(trainix), Xtrain(trainix, :), sopts);
        [predicted_label, accuracy, probs] = svmpredict(ytest, Xtest, model, '-b 1');
        [prec, rec, ap]= computepr(ytest, probs(:,posix), 'a', false);
        avap = avap + ap;
    end
    avap= avap/folds;
    cv_acc(i) = avap;
end

%# contour plot of parameter selection
[~,idx] = max(cv_acc);
contour(C, gamma, reshape(cv_acc,size(C))), colorbar
hold on
plot(C(idx), gamma(idx), 'rx')
text(C(idx), gamma(idx), sprintf('Acc = %.2f %%',cv_acc(idx)), ...
    'HorizontalAlign','left', 'VerticalAlign','top')
hold off
xlabel('log_2(C)'), ylabel('log_2(\gamma)'), title('Cross-Validation Accuracy')

best_C = 2^C(idx);
best_gamma = 2^gamma(idx);

model = svmtrain(ytrain, Xtrain, sprintf('-b 1 -c %f -g %f', best_C, best_gamma));
[predicted_label, accuracy, probs] = svmpredict(ytest, Xtest, model, '-b 1');
[prec, rec, ap]= computepr(ytest, probs(:,posix), 'a', false);
ap

%[predicted_label, accuracy, probs] = svmpredict(y, X, model, '-b 1');
kernelSVMScores= probs(:,posix);
%dlmwrite('kernelscores.eig.txt', kernelSVMScores, ' ');
%% train model linear SVM

N= size(Xtrain, 1);
folds = 5;
C = -5:2:10;
cv_acc = zeros(numel(C),1);
for i=1:numel(C)
    avap=0;
    for j = 1:folds
        ix = crossvalind('Kfold', N, folds);
        testix = (ix == j); trainix = ~testix;

        sopts= sprintf('-b 1 -c %f -t 0', 2^C(i));
        model = svmtrain(ytrain(trainix), Xtrain(trainix, :), sopts);
        [predicted_label, accuracy, probs] = svmpredict(ytest, Xtest, model, '-b 1');
        [prec, rec, ap]= computepr(ytest, probs(:,posix), 'a', false);
        avap = avap + ap;
    end
    avap= avap/folds;
    cv_acc(i) = avap;
end

%# contour plot of parameter selection
[~,idx] = max(cv_acc);
hold off
plot(C, cv_acc, 'r-')
hold on
text(C(idx), gamma(idx), sprintf('Acc = %.2f %%',cv_acc(idx)), ...
    'HorizontalAlign','left', 'VerticalAlign','top')
hold off
xlabel('log_2(C)'), ylabel('AP'), title('Cross-Validation Accuracy')

best_C = 2^C(idx);

model = svmtrain(ytrain, Xtrain, sprintf('-b 1 -c %f -t 0', best_C));
[predicted_label, accuracy, probs] = svmpredict(ytest, Xtest, model, '-b 1');
[prec, rec, ap]= computepr(ytest, probs(:,posix), 'a', false);
ap
linearSVMScores= probs(:,posix);
w = (model.sv_coef' * full(model.SVs));
bias= -model.rho;
predictions = Xtrain * w' + bias;
%predictions = X * w' + bias;
%dlmwrite('linearscores.eig.txt', predictions, ' ');
%% visualize results
clear aps;

% first show all measures individually
[prec, rec, aps(1)]= computepr(ytest, Xtest(:,1), 'a', false);
d1p{times}= prec; d1r{times}= rec;
[prec, rec, aps(2)]= computepr(ytest, Xtest(:,2), 'a', false);
d2p{times}= prec; d2r{times}= rec;
[prec, rec, aps(3)] = computepr(ytest, Xtest(:,3), 'a', false);
d3p{times}= prec; d3r{times}= rec;
[prec, rec, aps(4)] = computepr(ytest, Xtest(:,4), 'a', false);
d4p{times}= prec; d4r{times}= rec;
[prec, rec, aps(5)] = computepr(ytest, Xtest(:,5), 'a', false);
d5p{times}= prec; d5r{times}= rec;
[prec, rec, aps(6)] = computepr(ytest, Xtest(:,6), 'a', false);
d6p{times}= prec; d6r{times}= rec;

% average
[prec, rec, aps(8)] = computepr(ytest, mean(Xtest,2), 'a', false);
d8p{times}= prec; d8r{times}= rec;

% kernel SVM
[prec, rec, aps(9)] = computepr(ytest, kernelSVMScores, 'a', false);
d9p{times}= prec; d9r{times}= rec;

% linear SVM
[prec, rec, aps(10)] = computepr(ytest, linearSVMScores, 'a', false);
d10p{times}= prec; d10r{times}= rec;

% Nearest Neighbor
[prec, rec, aps(11)] = computepr(ytest, NNScores, 'a', false);
d11p{times}= prec; d11r{times}= rec;

% Naive Bayes
O1 = NaiveBayes.fit(Xtrain,ytrain);
C1 = O1.posterior(Xtest);
[prec, rec, aps(12)] = computepr(ytest, C1(:,2), 'a', false);
d12p{times}= prec; d12r{times}= rec;

aps
allaps=[allaps; aps];
end

%%

apstd= std(allaps)
apmean= mean(allaps)

subplot(121);
hold off;
plot(mean(horzcat(d1r{:}),2),mean(horzcat(d1p{:}),2),'-k.');
hold on;

plot(mean(horzcat(d2r{:}),2),mean(horzcat(d2p{:}),2),'-r.');
plot(mean(horzcat(d3r{:}),2),mean(horzcat(d3p{:}),2),'-g.');
plot(mean(horzcat(d4r{:}),2),mean(horzcat(d4p{:}),2),'-b.');
plot(mean(horzcat(d5r{:}),2),mean(horzcat(d5p{:}),2),'-m.');
plot(mean(horzcat(d6r{:}),2),mean(horzcat(d6p{:}),2),'-c.');
legend('Compactness', 'Symmetry', 'Global Convexity', 'Local Convexity', 'Smoothness', 'Recurrence', 'Location', 'NorthEast');
grid;
axis([0 1 0 1]);
title('Individual measures');

subplot(122);
hold off;
plot(mean(horzcat(d8r{:}),2),mean(horzcat(d8p{:}),2),'-k.');
hold on;

plot(mean(horzcat(d9r{:}),2),mean(horzcat(d9p{:}),2),'-g.');
plot(mean(horzcat(d10r{:}),2),mean(horzcat(d10p{:}),2),'-r.');
plot(mean(horzcat(d11r{:}),2),mean(horzcat(d11p{:}),2),'-b.');
plot(mean(horzcat(d12r{:}),2),mean(horzcat(d12p{:}),2),'-c.');
legend('Average', 'Kernel SVM', 'Linear SVM', 'Nearest Neighbor', 'Naive Bayes', 'Location', 'SouthEast');
grid;
axis([0 1 0 1]);
title('Measure combinations');

% %%
% subplot(121);
% x= mean(allaps(:,1:6));
% h= bar(x);
% bar_child=get(h,'Children');
% set(bar_child,'CData',x);
% set(gca,'xtick',[])
% title('Average precision');
%
% subplot(122);
% x= mean(allaps(:,8:end));
% h= bar(x);
% bar_child=get(h,'Children');
% set(bar_child,'CData',x);
% set(gca,'xtick',[])
% title('Average precision');
%
% h = bar('v6',Y);
% set(h(1),'facecolor','red') % use color name
% set(h(2),'facecolor',[0 1 0]) % or use RGB triple
% set(h(3),'facecolor','b') % or use a color defined in the help for PLOT
