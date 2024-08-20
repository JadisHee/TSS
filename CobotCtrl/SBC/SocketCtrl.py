import xml.etree.ElementTree as ET

class CommunicateData:
    
    TypeData = 2

    Command_Data = 1

    Error_Data = ''


    # def RecvData(self):
    #     Message = ET.Element("Message")
    #     Type = ET.SubElement(Message,"Type")
    #     Command = ET.SubElement(Message,"Type")

    #     Type.text = str(self.TypeData)
    #     Command.text = str(self.Command_Data)



    def XmlData(self):
        Message = ET.Element("Message")
        Type = ET.SubElement(Message,"Type")
        Command = ET.SubElement(Message,"Command")
        ErrorMsg = ET.SubElement(Message,"ErrorMsg")

        Type.text = str(self.TypeData)
        Command.text = str(self.Command_Data)
        ErrorMsg.text = self.Error_Data

        xml = ET.tostring(Message,encoding="unicode")
        return xml