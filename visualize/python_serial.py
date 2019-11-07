import settings
import serial
import re
ser = serial.Serial(settings.SERIAL_PORT, 57600)
print("connected to: " + ser.portstr)
line_temp = ""

def read_all_filtered_out():
    global ser
    line_temp = ""
    attributes = []
    data = {}
    while len(attributes) < 10:
        for line in ser.read():
            # print(chr(line)+"("+str(line)+")",end='')
            # print(chr(line),end='')
            if line == 0xa :
                temp_attributes = line_temp.split(":")
                attributes = []
                for attribute in temp_attributes:
                    for sub_attribute in attribute.split("#"):
                        attributes.append(sub_attribute)
                if len(attributes) < 10 :
                    print(line_temp)
                    #clear if no collect char from a 1st char or not collect all data
                    line_temp = ""
                    break
                if attributes[0] != "Index" and attributes[2] != "DEL" \
                    and attributes[4] != "ACC" and attributes[6] != "GYR" \
                    and attributes[8 != "FIL"]:
                    #clear if no collect char from a 1st char or not collect all data
                    line_temp = "" 
                    break
                    pass
                accel_angle = attributes[5].split(',')
                gyro_angle = attributes[7].split(',')
                angle = attributes[9].split(',')
                if len(accel_angle) + len(gyro_angle) + len(angle) < 9 :
                    break
                data = {
                    "result": 0,
                    "id": attributes[1],
                    "delta_time":attributes[3],
                    "gyro_angle_x":gyro_angle[0],
                    "gyro_angle_y":gyro_angle[1],
                    "gyro_angle_z":gyro_angle[2],
                    "accel_angle_x":accel_angle[0],
                    "accel_angle_y":accel_angle[1],
                    "accel_angle_z":accel_angle[2],
                    "angle_x":angle[0],
                    "angle_y":angle[1],
                    "angle_z":angle[2]
                }
                # print(attributes)
                print(data)
                print()
                line_temp = ""
            else :
                line_temp+=chr(line)
    return data
def terminate():
    global ser 
    ser.close()
if __name__ == '__main__':
    while True:
        for line in ser.read():
            # print(chr(line)+"("+str(line)+")",end='')
            # print(chr(line),end='')
            if line == 0xa :
                temp_attributes = line_temp.split(":")
                attributes = []
                for attribute in temp_attributes:
                    for sub_attribute in attribute.split("#"):
                        attributes.append(sub_attribute)
                if len(attributes) < 10 :
                    print(line_temp)
                    #clear if no collect char from a 1st char or not collect all data
                    line_temp = ""
                    break
                if attributes[0] != "Index" and attributes[2] != "DEL" \
                    and attributes[4] != "ACC" and attributes[6] != "GYR" \
                    and attributes[8 != "FIL"]:
                    #clear if no collect char from a 1st char or not collect all data
                    line_temp = "" 
                    break
                    pass
                accel_angle = attributes[5].split(',')
                gyro_angle = attributes[7].split(',')
                angle = attributes[9].split(',')
                if len(accel_angle) + len(gyro_angle) + len(angle) < 9 :
                    break
                data = {
                    "result": 0,
                    "id": attributes[1],
                    "delta_time":attributes[3],
                    "gyro_angle_x":gyro_angle[0],
                    "gyro_angle_y":gyro_angle[1],
                    "gyro_angle_z":gyro_angle[2],
                    "accel_angle_x":accel_angle[0],
                    "accel_angle_y":accel_angle[1],
                    "accel_angle_z":accel_angle[2],
                    "angle_x":angle[0],
                    "angle_y":angle[1],
                    "angle_z":angle[2]
                }
                # print(attributes)
                print(data)
                print()
                line_temp = ""
            else :
                line_temp+=chr(line)
    ser.close()