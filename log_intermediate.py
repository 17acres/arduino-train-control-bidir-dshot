from serial import Serial
from datetime import datetime

#remember the comma after each value (to help framing)
functable = {
    'DUMP_U32': lambda bytes: int.from_bytes(bytes,signed=False),
    'DUMP_U16': lambda bytes: int.from_bytes(bytes,signed=False),
    'DUMP_U8': lambda bytes: int.from_bytes(bytes,signed=False)
}


def main():
    dataFormat = []
    with open("src/main.cpp") as mainh:
        inPacket = False;
        for line in mainh:
            if("dataPacket[] = {" in line):
                inPacket = True;
            elif("0, 0, 0" in line):
                inPacket = False;
            elif(inPacket):
                line = line.split("//")[0]
                line = line.strip()
                splitted = line.split("(",1)
                signed = "DUMP_S" in line.split("(")[0] 
                params = splitted[1][0:-2].rsplit(",",1)
                
                name = params[0]
                res = float(params[1])
                dataFormat.append((signed,name,res))

    
    ser=Serial(port='/dev/ttyACM0',timeout=1)
    wh=open("data/auto"+datetime.now().strftime("%Y-%M-%d_%H-%M-%S")+".csv","w")
    header = ",".join([name for macro,name, res in dataFormat])
    print(header,file=wh)
    print(header)

    while(ser.is_open):
        line = ser.read_until(b"\0\0\0\0\0");
        splitted = line.split(b',')[:-1]
        if len(splitted) !=len(dataFormat) or max([len(chunk) for chunk in splitted])>4: #more than 4 bytes per
            if max([len(chunk) for chunk in splitted])>4:
                print(splitted[0].split(b'\r')[0].decode())
            continue
        printvalues = [str(int.from_bytes(bytes,signed=[dataFormat[index][0]])*dataFormat[index][2]) for index, bytes in enumerate(splitted)]
        print(",".join(printvalues),file=wh)
        print(",".join(printvalues))

if __name__ =="__main__":
    main()