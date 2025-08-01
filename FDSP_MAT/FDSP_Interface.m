clear
clc

s = serialport("COM8", 115200);
s.Timeout = 1000;
byteCounter = 1;
COM_START_SEND_SIGNAL = uint8('10');
ANS_MATLAB_READY_RX = uint8([0xAA, 0x55, 0x5A, 0xA5, 0xBB, 0xCC, ...
                             0x81, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, ...
                             0xFF, 0xFF, 0x0A, 0xFF]);

FlagGiveSignal = logical(false);
FlagExtracteSample = logical(false);
BufferPackets = [];
Signals = [];
IndexBufPack = 0;
IndexSampleSignal = 0;
ScalsePow = 0;
ScaleCalculate = int32(0);
NumPacketReceive = 0;
indexReadExter = 0;
CountSignal = 0;
SampleRate = 0;



while true
    
    if s.NumBytesAvailable >= 16
        data = read(s, 16, "uint8");
        if(FlagGiveSignal == true  &&  data(6) == hex2dec('91'))
            NumPacketReceive = NumPacketReceive + 1;
            for i = 1:16
                IndexBufPack  = IndexBufPack + 1;
                BufferPackets(IndexBufPack) = data(i);
            end
            if(NumPacketReceive == NumberSamples)
                FlagExtracteSample = true;
                IndexSampleSignal = 1;
                ScaleCalculate = int32(0);
                CountSignal = CountSignal + 1;
            end
        elseif(data(6) == hex2dec('15'))
            clc;
            byteCounter = 0;
            for i = 1:length(data)
                fprintf("Byte %d: 0x%s\n", byteCounter, dec2hex(data(i), 2));
                byteCounter = byteCounter + 1;
            end
         
            if(data(1) == hex2dec('AA')  &&  data(2) == hex2dec('55')  &&  data(3) == hex2dec('5A')  &&  ...
               data(4) == hex2dec('A5')  &&  data(5) == hex2dec('BB')  &&  data(6) == hex2dec('15')) 

               CheckSum = uint16(0);
               for index = 1 : 14
                   CheckSum = CheckSum + data(index);
               end
               CalCheckSum = bitor(bitshift(uint16(data(15)), 8), uint16(data(16)));
               if(CheckSum == CalCheckSum)
                   if(data(7) == hex2dec('10'))
                        NumberSamples = bitor( ...
                            bitor(bitshift(uint32(data(11)), 24), bitshift(uint32(data(12)), 16)), ...
                            bitor(bitshift(uint32(data(13)), 8), uint32(data(14))) ...
                            );
                        NumSignal = data(8);
                        ScalsePow = data(5);
                        SampleRate = bitor(bitshift(uint16(data(9)), 8), uint16(data(10)));

                        if(data(7) == hex2dec(COM_START_SEND_SIGNAL))
                            IndexBufPack = 0
                            NumPacketReceive = 0;
                            FlagGiveSignal = true;
                            write(s, ANS_MATLAB_READY_RX, "uint8");
                        end
                   end
                   if(data(7) == hex2dec('11'))
                        FlagGiveSignal = false;
                   end     
               end 
            end
        end
    end
    
    if(FlagExtracteSample == true)
        FlagExtracteSample = false;
        for indexReadExter = 1:IndexBufPack
            if(BufferPackets(indexReadExter  ) == hex2dec('AA')  && ...
               BufferPackets(indexReadExter+1) == hex2dec('55')  && ...
               BufferPackets(indexReadExter+2) == hex2dec('5A')  && ...
               BufferPackets(indexReadExter+3) == hex2dec('A5')  && ...
               BufferPackets(indexReadExter+4) == hex2dec('BB')  && ...
               BufferPackets(indexReadExter+5) == hex2dec('91'))     
                
                
                
                ScaleCalculate = bitor( ...
                        bitor(bitshift(int32(BufferPackets(indexReadExter+6)), 24), bitshift(int32(BufferPackets(indexReadExter+7)), 16)), ...
                        bitor(bitshift(int32(BufferPackets(indexReadExter+8)), 8), int32(BufferPackets(indexReadExter+9))) ...
                        );
                temp = double(ScaleCalculate) / (10^ ScalsePow);
                Signals(CountSignal, IndexSampleSignal) = double(temp);
                IndexSampleSignal = IndexSampleSignal + 1;

            end
        end

        byteCounter = 0;
        for i = 1:length(Signals)
            fprintf("Byte %d: %f\n", byteCounter, Signals(i));
            byteCounter = byteCounter + 1;
        end


if CountSignal >= 2
    disp(Signals);
    colors = ['r', 'g', 'b', 'k', 'm', 'c', 'y'];
    TimSample = double(0:NumberSamples-1) / double(SampleRate);

    for i = 1:CountSignal
        colorIndex = mod(i-1, length(colors)) + 1;  % دور بزن روی رنگ‌ها اگر زیاد شدند
        %%plot(TimSample, Signals(i, :), colors(i));
        stem(TimSample, Signals(i, :), colors(i));
        hold on;
    end 

    xlabel('Sample Index');
    ylabel('Amplitude');
    title('Overlayed Signals');
    legendStrings = arrayfun(@(x) ['Signal ' num2str(x)], 1:CountSignal, 'UniformOutput', false);
    legend(legendStrings);
    grid on;
    hold off;
    datacursormode on;
    CountSignal = 0;

end



    end



    pause(0.01);
end









