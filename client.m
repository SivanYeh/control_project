function client(port)
%   provides a menu for accessing PIC32 motor control functions
%

%
%   Input Arguments:
%       port - the name of the com port.  This should be the same as what
%               you use in screen or putty in quotes ' '
%
%   Example:client('/dev/
%       client('/dev/ttyUSB0') (Linux/Mac)
%       client('COM3') (PC)
%
%   For convenience, you may want to change this so that the port is hardcoded.
   
% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, hardware flow control
% wait up to 120 seconds for data before timing out
mySerial = serial(port, 'BaudRate', 230400, 'FlowControl', 'hardware','Timeout',120); 
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));                                 

has_quit = false;
% menu loop
while ~has_quit
    fprintf('PIC32 MOTOR DRIVER INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf(' a: Read current sensor (ADC Counts)\t b: Read Current Sensor(mA)\n c: Read encoder (Counter)\t\t d: Motor Angle in degrees\n e: reset encoder\t\t\t f: Set PWM (-100 to 100)\n g: Set gains\t\t\t\t h: get gains\n q: Quit\n');
    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
     
    % send the command to the PIC32
    fprintf(mySerial,'%c\n',selection);
   
    % take the appropriate action
    switch selection
        case 'a'
            counts = fscanf(mySerial,'%d');
            fprintf('The ADC counts is %d\n', counts)
        case 'b'
            counts = fscanf(mySerial,'%d');
            fprintf('The current is %d mA\n', counts)
        case 'c'
            counts = fscanf(mySerial,'%d');
            fprintf('The motor angle is %d counts.\n', counts)        
        case 'd'                         % example operation
            deg = fscanf(mySerial,'%d');
            fprintf('The motor angle is %d degrees.\n', deg)
        case 'e'
            counts = fscanf(mySerial,'%d');
            fprintf('The motor angle is %d counts.It is reset\n', counts)
        case 'g'
            selection = input('\nEnter Current proportional gain, Kp', 's')
            fprintf(mySerial,'%s\n',selection);
            selection = input('\nEnter Current integral gain, Ki', 's')
            fprintf(mySerial,'%s\n',selection);
            fprintf('The Current gains have been set.')
        case 'h'
            gain = fscanf(mySerial,'%s');
            fprintf('The gains Kp and Ki are %s repectively.\n',gain)

        case 'q'
            has_quit = true;             % exit client
        
        case 'r'
            temp = fscanf(mySerial,'%s');
            fprintf('The PIC32 controller mode is %s\n',temp);
             
        case 'p'
            counts = fscanf(mySerial,'%d');
            fprintf('PWM has been set to %d\n', counts)        
        case 'f'
            selection = input('\nENTER COMMAND: ', 's');
            fprintf(mySerial,'%s\n',selection);
            counts = fscanf(mySerial,'%d');
            fprintf('PWM has been set to %d\n', counts)  
                        
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
end

end
