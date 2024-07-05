path = 'C:\Users\zekim\Downloads\undergrad project\project\final_test2\Test33';  %change !....

myfolder = 'figs_test33' ;   % new folder name 
folder = mkdir([path,filesep,myfolder]) ;
path  = [path,filesep,myfolder] ;
for k = 1:45
    figure(k)
    temp=[path,filesep,'fig',num2str(k),'.fig'];
    saveas(gca,temp);
end