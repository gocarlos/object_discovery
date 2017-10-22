function [prec, rec, ap] = computepr(gt, confidence, classname, draw)
  if nargin < 3, classname= 'a class'; end
  if nargin < 4, draw = true; end

  [so,si]=sort(-confidence);
  tp=gt(si)>0;
  fp=gt(si)<0;

  fp=cumsum(fp);
  tp=cumsum(tp);
  rec=tp/sum(gt>0);
  prec=tp./(fp+tp);

  ap=VOCap(rec,prec);

  if draw
      % plot precision/recall
      plot(rec,prec,'-');
      grid;
      xlabel 'recall'
      ylabel 'precision'
      title(sprintf('class: %s, AP = %.3f', classname, ap));
  end

function ap = VOCap(rec,prec)
  mrec=[0 ; rec ; 1];
  mpre=[0 ; prec ; 0];
  for i=numel(mpre)-1:-1:1
      mpre(i)=max(mpre(i),mpre(i+1));
  end
  i=find(mrec(2:end)~=mrec(1:end-1))+1;
  ap=sum((mrec(i)-mrec(i-1)).*mpre(i));
