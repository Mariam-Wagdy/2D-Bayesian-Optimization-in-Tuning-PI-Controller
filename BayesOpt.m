clc;
clear;


var1 = optimizableVariable('x1',[1,10],'Type','integer');
var2 = optimizableVariable('x2',[1,10],'Type','integer');

fun = @(var)test_fun(var.x1,var.x2); %Create Function Handles
%results = bayesopt(fun,[var1, var2],'AcquisitionFunctionName','expected-improvement-plus');
results = bayesopt(fun,[var1, var2],'AcquisitionFunctionName','expected-improvement-plus',...
    'MaxObjectiveEvaluations',30);





