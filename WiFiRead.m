function [] = WiFiRead(counter,deltaT,voltage)

% counter : It will make sure that the sent serialdata is new or not
% deltaT : deltaT that is sent will multiply by 5000 if the voltage is maximum (255) 
% voltage : For turning the input volatge should be less than 50 volt and the turn
%timeStep is 11000 that is the amount of time required to turn 90 degree

% Open the serial port
s = serial('COM8', 'BaudRate', 9600); 
   fopen(s);

%Data Order = [counter ,deltaT , Voltage];
counterS=num2str(counter, '%02d');
deltaTS=num2str(deltaT, '%02d');
voltageS=num2str(voltage, '%02d');

% Data is put together as a String with "$" as a footer
finalData = [counterS,',' , deltaTS,',' , voltageS,'$'];
fprintf(s, finalData);

% close the serial port
   fclose(s);
   delete(s);
   clear s;

end