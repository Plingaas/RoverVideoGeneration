import numpy as np

def readDataset(filename, offsetX=0, offsetY=0, offsetHeading=0):
    print(f"Reading {filename}.")
    frames = []
    with open(filename, 'r') as file:
        while True:
            line = file.readline()
            if (len(line) == 0):
                break
            data = line.split(";")
            t = data[0]
            posX = float(data[1])
            posY = float(data[2])
            heading = float(data[3]) + offsetHeading # in radians
            lidar = data[4].split(":")
            lidarX, lidarY = [], []

            # Rover driving forward is y, but for lidar forward is x
            _posX = posX * np.sin(offsetHeading) + posY * np.cos(offsetHeading) + offsetX
            _posY = -(posX * np.cos(offsetHeading) - posY * np.sin(offsetHeading)) + offsetY
            
            for i in range(len(lidar)-1):
                x, y = map(float, lidar[i].split(","))
                _x = x*np.cos(heading) - y*np.sin(heading) + _posX
                _y = x*np.sin(heading) + y*np.cos(heading) + _posY
                lidarX.append(_x)
                lidarY.append(_y)
            
            frames.append((t,_posX,_posY,heading,lidarX,lidarY))
    return frames

def writeDataset(filename, frames):
    with open(filename, 'w') as file:
        for frame in frames:
            file.write(f"{frame[0]};{frame[1]};{frame[2]};{frame[3]};")
            for i in range(len(frame[4])):
                file.write(f"{frame[4][i]},{frame[5][i]}:")
            file.write("\n")