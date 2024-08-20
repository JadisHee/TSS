import socket
import xml.etree.ElementTree as ET

duco_antenna_ip = '192.168.1.225'
duco_antenna_port = 9998

# command_data = 1
# Type_data = 3
# Num_data = 2

def simu_ctrler(command_data,Type_data,Num_data):
    Message = ET.Element("Message")
    
    Command = ET.SubElement(Message,"Command")
    ProductType = ET.SubElement(Message,"ProductType")
    ProductNum = ET.SubElement(Message,"ProductNum")

    Command.text = str(command_data)
    ProductType.text = str(Type_data)
    ProductNum.text = str(Num_data)

    xml = ET.tostring(Message,encoding="unicode")
    return xml

if __name__ == "__main__":
    ctrler_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    ctrler_client.connect((duco_antenna_ip,duco_antenna_port))

    for i in range(6):
        send_xml = simu_ctrler(i+1,5,1)

        ctrler_client.send(str(send_xml).encode('utf-8'))

        recv_data = ctrler_client.recv(1024)

        print(recv_data)